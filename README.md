![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/verify/badge.svg)

# Polyglot: a chip that learns protocols after it is made

Polyglot is an open-source, general-purpose **protocol emulator ASIC** for the
[Jane Street Protocol Emulator competition](https://tinytapeout.com/competitions/jane-street-tt-protocol-emulator/)
on Tiny Tapeout (IHP 130 nm CMOS5L, 8x4 tiles).

It contains **no UART, no SPI, no I2C**. It has four tiny cycle-exact
processors whose instructions are all about pins, shift registers and time.
Every protocol is a program, so the chip can be taught protocols nobody
planned for.

```
              ┌─────────────────── 32 x 24-bit shared IMEM ──────────────────┐
 RP2350 ─SPI─►│ host port: load / start / single-step / inject / FIFOs / debug│
              └──┬──────────────┬──────────────┬──────────────┬──────────────┘
           ┌─────▼────┐   ┌─────▼────┐   ┌─────▼────┐   ┌─────▼────┐
           │  core 0  │   │  core 1  │   │  core 2  │   │  core 3  │   each: X, Y, ISR, OSR,
           │ div  CRC │   │ div  CRC │   │ div  CRC │   │ div  CRC │   16-bit divider, CRC-16,
           │ TX⇅RX FIFO│  │ TX⇅RX FIFO│  │ TX⇅RX FIFO│  │ TX⇅RX FIFO│  side-set, open-drain
           └─────┬────┘   └─────┬────┘   └─────┬────┘   └─────┬────┘
                 └───── 8 flags ─── timer ─── pin arbitration ───┘
                         ui[4:0]  uo[6:0]  uio[7:0] (bidirectional)
```

## What it speaks today (all in `programs/`, all tested)

| protocol | words | notes |
|---|---|---|
| UART TX / RX | 4 / 8 | 8N1, any baud via divider, framing-error flag |
| Modbus RX | 9 | **hardware CRC-16/MODBUS** at line rate (one `.crc` line) |
| SPI master | 11 | mode 0, full duplex, automatic chip-select |
| I2C master | 19 | open-drain, **repeated START, clock stretching**, ACK/NACK |
| WS2812 / NeoPixel | 4 | *never planned*: exact 400/800 ns pulses |
| 10BASE-T Manchester | 6 | *never planned*: differential, **10 Mbit/s at 60 MHz** |
| USB 1.1 low-speed TX | 30 | *never planned*: NRZI, **bit stuffing**, EOP, all in software |

Four can run at once. The test suite runs UART + SPI + WS2812 + a pulse meter
written *inside the test* concurrently.

## What makes the architecture different

* **A hard timing contract**: every instruction takes exactly `1 + delay`
  ticks. Stalls change nothing but side-set pins. These properties are
  **formally proven** (unbounded k-induction), not just simulated.
* **Free side-set**: up to 3 pins change in parallel with any instruction,
  with a per-instruction enable bit so no delay bits are sacrificed.
* **Open-drain mode**: I2C, SMBus and 1-Wire with no extra instructions.
* **`wait match`**: wait for a multi-pin pattern (`(pins & Y) == X`), for
  example USB SE0 or bus-idle, in one instruction.
* **Serial CRC engine per core** with programmable polynomial, fed for free
  by 1-bit shifts (Modbus, USB, CCITT, SMBus PEC, 1-Wire, CRC-5).
* **Right-justified pushes**: bytes arrive as bytes, whatever the shift direction.
* **`mov pins, !pins`**: one-instruction NRZI / toggle encoding.
* **On-silicon debugger**: stop, single-step, inject any instruction, read
  X/Y/CRC/PC.
* **Glitch-free host FIFO reads**: a word is consumed only after it has been
  completely clocked out.

## Verification (details: [docs/verification.md](docs/verification.md))

| layer | result |
|---|---|
| lock-step co-simulation, RTL vs golden model, random programs + host traffic + pins, ~55 fields/cycle | **2.5 M cycles on final RTL, 0 mismatches**, 55/55 opcode bins |
| mutation testing of the checker | **21/21** injected bugs caught |
| formal (SymbiYosys + Z3, unbounded) | timing contract, FIFO and SPI invariants **proven** (and found a latent SPI bug) |
| protocol tests through the real SPI port (cocotb) | **9/9** |
| same tests on the IHP gate-level netlist | **9/9** |
| Verilator `-Wall` lint | clean |

## Physical (pre-layout, Yosys + IHP CMOS5L)

About 354,000 um^2 of cells and 2,567 flops in Yosys (36% of the 8x4 tiles). The real LibreLane flow adds buffering: the first full-flow run of the original 64-word version measured 67% placement utilization, which is why the instruction memory was halved to 32 words (expected about 50%).
Critical-path estimate (NLDM, x1 cells, before OpenROAD upsizing): 12.8 ns
typical / 20 ns slow corner. The flow targets 50 MHz, and 60 MHz is expected
at typical silicon.

## Quick start

```sh
pip install cocotb pytest                          # + iverilog
cd tools
python -m polyglot.asm ../programs/uart_tx.pga     # listing
python -m polyglot.run ../programs/uart_tx.pga --push 0x48,0x69 --vcd uart.vcd   # simulate, no hardware
cd ../test && make                                 # protocol tests on the RTL
python3 cosim/run_cosim.py --seeds 50 --cov        # co-simulation
```

On the demo board: `firmware/polyglot_mp.py` (MicroPython, RP2350).

## Repository

| path | contents |
|---|---|
| `src/` | RTL: `pg_core.v` (core), `pg_cluster.v` (IMEM, FIFOs, pins, registers), `pg_host_spi.v`, `pg_fifo.v`, `project.v` (top) |
| `tools/polyglot/` | assembler (`asm.py`), cycle-accurate golden model (`sim.py`), simulator CLI (`run.py`), host driver (`host.py`) |
| `programs/` | example programs |
| `test/` | cocotb protocol tests, `cosim/` co-simulation and mutation testing |
| `formal/` | SymbiYosys proofs |
| `synth/`, `tools/gl/` | area / netlist / timing estimation, gate-level model helper |
| `firmware/` | RP2350 MicroPython host driver |
| `docs/` | [ISA](docs/isa.md), [registers & SPI](docs/registers.md), [writing a new protocol](docs/programming.md), [verification write-up](docs/verification.md), [datasheet](docs/info.md) |

## Pinout

| pin | function |
|---|---|
| `ui[4:0]` | general inputs (pin space 16-20) |
| `ui[7:5]` | host SPI CS_n / SCK / MOSI |
| `uo[6:0]` | general outputs (pin space 8-14) |
| `uo[7]` | host SPI MISO |
| `uio[7:0]` | general bidirectional (pin space 0-7) |

## AI use

Designed and built with an AI assistant (Claude). How that was made
trustworthy, and the bugs it introduced and then caught, are described in
[docs/verification.md](docs/verification.md#use-of-ai-tools).

## License

Apache-2.0
