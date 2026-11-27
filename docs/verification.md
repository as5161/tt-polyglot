# Verification write-up

Polyglot's verification is built around one idea: **the instruction set is
defined by an executable golden model, and the RTL is continuously proven to
agree with it.** That model, the assembler and the host driver are the same
Python code users run to develop new protocols, so what is verified is what
gets shipped.

```
         programs/*.pga ── pgasm ──┐
                                   ▼
  ┌───────────────┐   identical   ┌──────────────┐
  │ golden model  │◄── stimulus ──│ RTL (iverilog)│      formal (SymbiYosys)
  │ tools/sim.py  │               │   src/*.v     │◄──── k-induction proofs
  └──────┬────────┘               └──────┬───────┘      of the timing contract
         └──── ~55 state fields compared every cycle ───┘
                    ▲
       mutation testing proves the checker can fail
```

Six layers, each catching a different class of bug:

| # | layer | what it proves | tool | where |
|---|---|---|---|---|
| 1 | Lock-step co-simulation | RTL == golden model, cycle by cycle, for random programs, configs, host traffic and pins | iverilog + Python | `test/cosim/run_cosim.py` |
| 2 | Functional coverage | every opcode x variant executed, every stall type hit | Python | `--cov` |
| 3 | Mutation testing | the checker detects realistic bugs (21 injected, 21 killed) | Python | `test/cosim/mutate.py` |
| 4 | Formal (unbounded) | timing contract, counter bounds, FIFO and SPI invariants | SymbiYosys + Z3 | `formal/*.sby` |
| 5 | Protocol tests | real protocols work end-to-end through the SPI host port | cocotb | `test/test.py` |
| 6 | Gate-level simulation | the IHP-mapped netlist behaves like the RTL | cocotb + IHP cells | `make GATES=yes` |

## 1. Constrained-random lock-step co-simulation

`run_cosim.py` generates, for each seed:

* a random 32-word program (plus writes to the unmapped addresses 32-63), biased towards valid encodings, with 3% fully
  random words so reserved opcodes and odd field values are covered;
* random configuration of all four cores (dividers, wraps, shift directions,
  thresholds, CRC, open-drain, pin bases);
* random host traffic **every cycle** at a per-seed rate: FIFO pushes and pops,
  register reads with side effects, `EXEC` injection, PC writes, `CTRL`
  restart/divsync/enable, single-step, flag writes, reconfiguration while
  running, and **self-modifying IMEM writes**;
* a random walk on all 24 input pins at a per-seed toggle rate.

The same stimulus drives the RTL cluster (via `tb_cosim.v`) and
`tools/polyglot/sim.py`. After every clock the full architectural state is
compared: host read data, pin outputs and enables, flags, enables, and per
core PC, X, Y, ISR, OSR, CRC, both shift counts, delay counter, FIFO levels,
RX FIFO head and EXEC-pending. The testbench also asserts the
**fetch-pipeline invariant** (`IR == imem[PC]` for every core, every cycle).

Seeds vary the opcode mix (half the seeds focus on random subsets of
opcodes) and force pin-range overlaps on a quarter of seeds. Both changes
were added because mutation testing found holes (see 3).

**Result on the final RTL:** 500 seeds x 5,000 cycles = **2.5 M cycles**
(x4 cores) with zero mismatches, 55/55 opcode/variant coverage bins hit.
Earlier RTL revisions passed a further 700 seeds (3.5 M cycles). A 40-seed
smoke run follows every RTL edit.

## 2. Coverage

The model records which (opcode, variant) pairs actually *executed* and which
instruction types stalled:

```
coverage: 55/55 opcode/variant bins hit
  stall events by opcode: {'wait': 782494, 'in': 113572, 'out': 176545, 'push/pull': 259985}
cosim: 500/500 seeds passed, 2500000 cycles compared
```

## 3. Mutation testing: checking the checker

A green regression only means something if it can go red. `mutate.py`
injects 21 realistic single-line bugs into the RTL (off-by-one thresholds,
inverted conditions, priority swaps, missing auto-clear, wrong CRC tap,
FIFO depth, divider running while disabled, missing fetch bypass, ...) and
requires the co-simulation to fail on each.

The first run scored **17/19**. Two mutants survived:

* *push data not right-justified*: RX FIFO contents were only compared
  when the random host happened to pop them. Fix: compare the RX head every
  cycle.
* *side-set vs. OUT priority swapped*: random pin bases rarely overlapped.
  Fix: a "side" seed mode that forces overlaps.

After both fixes: **21/21 killed** (two more mutants were added for the
fetch pipeline).

## 4. Formal verification

SymbiYosys with Z3, `prove` mode (k-induction, so the results are
**unbounded**: they hold for all time, not just N cycles):

* **pg_core**: the timing contract that makes Polyglot a protocol engine:
  * a stalled instruction changes no architectural state (PC, X, Y, ISR, OSR,
    CRC, counts);
  * a completed instruction loads exactly its delay field;
  * every delay tick decrements by exactly one and executes nothing;
  * no tick and no EXEC means the core is frozen;
  * shift counts never exceed 16; the core never pushes into a full FIFO
    or pops an empty one; nothing has side effects unless issued.
* **pg_fifo**: occupancy bounds and pointer consistency.
* **pg_host_spi**: one bus operation per cycle; only FIFO reads can have
  side effects; bit counters bounded.

A vacuity check (weakening one assertion to be false) confirms the proofs
can fail.

**Formal found a real bug.** Induction on the SPI front-end produced a
counterexample where CS is released and re-asserted within two clocks of a
word boundary. A pending prefetch then lands during the next header and
leaves a stale "valid FIFO word" flag. In the current design that flag is
always overwritten before use, so the bug was latent, but it was exactly
the sort of thing that bites after a later edit. Fixed by flushing the read
pipeline on CS high.

## 5. Protocol tests (cocotb, top level)

Every test drives the **real SPI host port** bit by bit, as the RP2350
does: assemble `programs/*.pga`, load IMEM, configure, start. Independent
Python protocol models then check the pins:

| test | checks |
|---|---|
| host interface | ID, IMEM/register read-back, EXEC into a stopped core, single-step |
| UART TX | 8N1 decode of 6 bytes incl. 0x00/0xFF |
| UART RX + Modbus CRC | 6-byte frame; hardware CRC equals reference CRC-16/MODBUS; residue 0 after appending CRC; framing error raises flag 0 |
| SPI master | mode-0 slave model, full duplex, auto chip-select, CS returns high |
| I2C master | EEPROM model with **clock stretching**: write 3 bytes, set pointer, **repeated START**, read 3 bytes (ACK, ACK, NACK+STOP), NACK from a missing address |
| WS2812 | high-pulse widths exactly 4 / 8 ticks, 48 bits decoded |
| Manchester / 10BASE-T | TD+/TD- complementary, a mid-bit transition on every bit, 48 bits decoded at **6 clocks/bit (10 Mbit/s at 60 MHz)** |
| USB low-speed TX | NRZI decode, **bit-stuffing** (0xFFFF payload), CRC16, EOP = 2 bits SE0 then J, done flag |
| four protocols at once | UART + SPI + WS2812 + a pulse meter **written inside the test** run concurrently on all four cores |

### Bugs found by the protocol tests

* **Lost RX data on every SPI read.** The host interface prefetched the next
  word at the end of every read and popped the FIFO, so the last pop of each
  frame was thrown away. The cluster-level co-simulation could not see this
  because it bypasses the SPI front-end. Fix: reads became side-effect-free
  peeks, and a FIFO word is consumed only after it has been completely
  transferred and was valid.
* Empty-FIFO reads returned uninitialised RAM (X in simulation, random bits
  on silicon). Now they return zeros.
* Several bugs in my own *programs* (label targets, USB slot timing off by a
  tick, I2C SDA changing on the same tick as SCL falling) were found by
  hand-timing every program in comments, then confirmed by the models.

## 6. Gate-level simulation

`synth/synth.ys` maps the design onto the IHP `sg13cmos5l` standard-cell
library, and the **entire cocotb suite passes on that netlist** (9/9).
Icarus 12 does not drive the `delayed_*` nets of the IHP cell models, so
`tools/gl/make_functional_models.sh` makes a functional copy for local runs.
CI's `gl_test` job additionally runs the suite on the final LibreLane
netlist.

## Physical estimates (pre-layout)

| metric | value |
|---|---|
| cells (IHP CMOS5L, typ, Yosys) | ~25,400 |
| flip-flops | 2,567 |
| cell area | 354,000 um^2 in Yosys (the real flow is larger, see below) |
| timing estimate (`tools/gl/mini_sta.py`, NLDM, x1 cells, buffer-tree model) | 12.8 ns typical (~78 MHz), 20.1 ns slow corner (~50 MHz) |
| flow target | `CLOCK_PERIOD` 20 ns (50 MHz). Upsizing in OpenROAD should improve on the x1-cell estimate |

Area-driven design decisions, measured with Yosys against the IHP library:

* A 16-bit, n-bit-per-cycle CRC cost 20,000 um^2 **per core**. Serial
  protocols feed CRCs one bit per instruction anyway, so the CRC became a
  1-bit engine fed automatically by 1-bit shifts: ~200 um^2.
* The fetch was pipelined (an instruction register with a host-write
  bypass) to take the IMEM read mux out of the execute path. It is
  architecturally invisible, which the co-simulation and an explicit
  invariant check prove. Estimated critical path: 15.4 ns to 14.3 ns (typical).
* The datapath feeding the shared shifter, CRC and justify units was moved
  out of the execute `always` block. This removed a false combinational loop
  that Verilator flagged (`UNOPTFLAT`) and shortened the critical path from
  14.3 ns to 12.8 ns (typical). The co-simulation, mutation and formal
  suites were rerun after the change.
* Dropping computed jumps (`out pc`) was measured to save only ~10%, so the
  ISA was kept intact.

## First runs through the real Tiny Tapeout flow

The GitHub `gds` action (LibreLane + OpenROAD) is the authoritative check, and
its first runs failed. Each failure taught something:

| run | symptom | cause | fix |
| --- | --- | --- | --- |
| 1-2 | `GPL-0302 target density too low`, then `GRT-0010 Instance _061735_ is not placed` | the IHP I/O pad library is passed to synthesis, and its output pads (`IOPadOut4mA/16mA/30mA`) are described as plain buffers without `dont_use`. ABC picked four 120 x 120 um pads as strong drivers for high-fanout nets; they can never be placed in a standard-cell row | `"EXTRA_EXCLUDED_CELLS": ["sg13cmos5l_IOPad*"]` in `src/config.json` |
| 3 | placement now completes, but at **67% utilization** (55,364 cells, 764,000 um^2), far above the 40% the quick Yosys estimate suggested | LibreLane's synthesis and buffering produce a much larger netlist than a bare `synth; abc` | instruction memory halved from 64 to 32 words (-23% area, same budget as an RP2040 PIO block); placement density back to the default 60% |
| 4-5 | log stops at line 19,322 and the step runs for hours | most likely not a hang (identical code passed this point in seconds in runs 1-3): the live log shows output in buffered chunks while detailed routing of a 67%-full design on three routing layers runs very slowly | the area reduction above |

Lesson: early area estimates from a plain Yosys run were 1.65x optimistic for
this flow. Budget for that from the start.

## How to run everything

```sh
cd test && make                                  # protocol tests, RTL
cd test && make GATES=yes PDK_ROOT=...           # gate level
python3 test/cosim/run_cosim.py --seeds 200 --cov   # lock-step co-simulation
python3 test/cosim/mutate.py                     # mutation score
cd formal && sby -f pg_core.sby                  # proofs (also pg_fifo, pg_host_spi)
```

## Use of AI tools

This project was designed and implemented with an AI assistant (Claude),
working in a sandbox with Icarus Verilog, Yosys, SymbiYosys/Z3 and cocotb.
The AI wrote the architecture, RTL, golden model, assembler, programs, tests
and this documentation.

What made the work trustworthy despite that was not the AI's confidence but
the structure. Every artefact the AI produced was checked by a different
artefact that could disagree with it:

* the AI-written RTL was checked against an AI-written model, which could
  share a misunderstanding. Hence the protocol tests check against
  *independent* protocol models written from the protocol specs, not from
  the ISA;
* the AI-written checker was itself attacked by mutation testing. Its first
  run exposed two blind spots;
* formal proofs over the RTL found a latent bug that both simulators missed;
* real bugs the AI introduced, and later caught, are listed above rather
  than hidden. The most serious (losing a byte on every SPI read) slipped
  past the 1.5 M-cycle cluster co-simulation because that harness
  bypassed the SPI front-end. That is a lesson about verification
  *boundaries*, not about randomness.
