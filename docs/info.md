## How it works

Polyglot is a **programmable protocol emulator**: four tiny cycle-exact
processors ("cores") whose instructions are all about pins, shift registers
and time. A protocol is a program, not a circuit, so the chip can learn
protocols that did not exist when it was made.

* 4 cores, shared 64 x 24-bit instruction memory (loaded over SPI)
* every instruction takes exactly `1 + delay` ticks (delay 0..31), and each core has a 16-bit clock divider
* per core: X/Y registers, 16-bit in/out shift registers with autopush/autopull,
  4-deep TX and RX FIFOs, a 16-bit serial CRC engine with programmable polynomial
* side-set on up to 3 pins in parallel with any instruction, open-drain mode,
  multi-pin pattern `wait`, NRZI-friendly `mov pins, !pins`, a free-running timer, 8 inter-core flags
* host debug: single-step, instruction injection, register read-back

Example programs: UART TX/RX, Modbus RX with hardware CRC, SPI master,
I2C master (repeated start, clock stretching), WS2812, 10 Mbit/s Manchester
(10BASE-T style), and USB 1.1 low-speed TX with NRZI and bit stuffing.

Full documentation: `docs/isa.md`, `docs/registers.md`, `docs/programming.md`,
`docs/verification.md` in the repository.

## How to test

1. Clock the project (50 MHz recommended; programs give `clkdiv` for 60 MHz).
2. Pulse reset, then talk SPI mode 0 on `ui[7]`=CS_n, `ui[6]`=SCK (<= clk/8),
   `ui[5]`=MOSI, `uo[7]`=MISO. Reading address `0xA5` returns `0x504701`.
3. Assemble a program on a PC (`python -m polyglot.asm programs/uart_tx.pga -o uart_tx.json`),
   load it with `firmware/polyglot_mp.py` on the demo board's RP2350, start the core,
   and push bytes. The UART appears on `uo[0]`.

The program must be reloaded after the project is deselected (state is lost).

## External hardware

Depends on the protocol: none for UART/SPI/WS2812 logic-level use. I2C
needs pull-ups on `uio[4]`/`uio[5]`. USB needs a USB Pmod (level-shifting
and series resistors), and 10BASE-T needs an Ethernet magnetics Pmod.
