# Polyglot instruction set

Polyglot has **four identical cores** sharing a **32 x 24-bit instruction memory**, the same budget as one Raspberry Pi RP2040 PIO block.
Each core is a tiny processor whose only job is to move bits between pins and
FIFOs at exactly the right clock cycle.

## Execution model (the timing contract)

* Each core advances on a **tick**, produced by its own 16-bit integer clock
  divider (`CLKDIV`; 0 or 1 = every system clock).
* **Every instruction takes exactly `1 + delay` ticks**, where `delay` (0..31)
  is encoded in the instruction, unless it **stalls**.
* Only five things can stall, and while stalled nothing changes except the
  side-set pins: `WAIT` (condition false), blocking `PULL` on an empty TX FIFO,
  blocking `PUSH` on a full RX FIFO, `OUT` with autopull on an empty FIFO, and
  `IN` with autopush into a full FIFO.
* **Side-set** pins are written on every tick the instruction is presented,
  including while it stalls. Side-set wins over a main-op write to the same pin.
* The fetch is pipelined but invisible: a taken jump costs nothing extra.
* Inputs pass a 2-flop synchroniser (2 cycles of fixed latency). Pin writes
  appear on the pads one cycle after the instruction executes.

These rules are formally proven on the RTL (see [verification.md](verification.md)).

## Architectural state per core

| state | width | notes |
|---|---|---|
| `PC` | 5 | wraps from `WRAP_TOP` to `WRAP_BOT` for free |
| `X`, `Y` | 16 | scratch / counters |
| `ISR` + count | 16 + 0..16 | input shift register; pushes are **right-justified** |
| `OSR` + count | 16 + 0..16 | output shift register; count 16 = empty |
| `CRC` | 16 | serial CRC engine, programmable polynomial |
| TX / RX FIFO | 4 x 16 each | host-facing |

Shared between cores: 8 event **flags** (host-visible), a 16-bit free-running
**TIME** counter and the pins.

## Pin space

All pin operands index a 32-entry pin space (indices wrap modulo 32):

| index | pins | direction |
|---|---|---|
| 0..7 | `uio[7:0]` | bidirectional (output enable = `PINDIRS`) |
| 8..14 | `uo[6:0]` | output only (reads return the driven value) |
| 15 | `uo[7]` | host MISO (read-only) |
| 16..20 | `ui[4:0]` | input |
| 21..23 | `ui[7:5]` | host SPI (read-only) |

Each core has independent bases for `OUT` (`OUT_BASE`, `OUT_COUNT` 0..16),
`SET` (`SET_BASE`, `SET_COUNT` 0..8), `IN` (`IN_BASE`) and side-set
(`SIDE_BASE`, 1..3 pins), plus a `JMP_PIN`. When several cores write the same
pin in one cycle, the highest-numbered core wins.

**Open-drain mode** (`OD` config bit): every value write to a pin becomes a
direction write. 0 drives low, 1 releases. This is how I2C, SMBus and 1-Wire
work, with no extra instructions.

## Encoding

```
 23    20 19 18  16 15   11 10                 0
+--------+--+------+-------+--------------------+
| opcode |SE| side | delay |      operands      |
+--------+--+------+-------+--------------------+
```

`SE` = side-set enable for this instruction (so side-set is optional per
instruction with no loss of delay bits). `side` = values for up to 3 side pins.

| op | mnemonic | operands `[10:0]` |
|---|---|---|
| 0 | `JMP cond, addr` | `[10:7]` cond, `[5:0]` address (`[4:0]` used with the 32-word IMEM) |
| 1 | `WAIT pol src idx` | `[10]` polarity, `[9:8]` source, `[4:0]` index |
| 2 | `IN src, n` | `[10:8]` source, `[3:0]` bit count (0 = 16) |
| 3 | `OUT dst, n` | `[10:8]` destination, `[3:0]` bit count |
| 4 | `PUSH` / `PULL` | `[10]` 1 = pull, `[9]` iffull/ifempty, `[8]` block |
| 5 | `MOV dst, op src` | `[10:8]` dst, `[7:6]` op, `[3:0]` src |
| 6 | `FLAG set/clr n` | `[10:9]` 0 = set, 1 = clear, `[2:0]` flag |
| 7 | `SET dst, imm8` | `[10:8]` dst, `[7:0]` immediate |
| 8 | `ADD x/y, simm8` | `[10]` 0 = X, 1 = Y, `[7:0]` signed immediate |
| 9..15 | `NOP` | reserved; execute as no-operation (verified) |

### JMP conditions

| code | syntax | jump if |
|---|---|---|
| 0 | *(none)* | always |
| 1 | `!x` | X == 0 |
| 2 | `x--` | X != 0, **then** X = X - 1 (always decrements) |
| 3 | `!y` | Y == 0 |
| 4 | `y--` | Y != 0, then Y = Y - 1 |
| 5 | `x!=y` | X != Y |
| 6 | `pin` | `JMP_PIN` high |
| 7 | `!osre` | OSR not empty (count < pull threshold) |
| 8 | `!pin` | `JMP_PIN` low |
| 9 | `txne` | TX FIFO not empty |
| 10 | `rxnf` | RX FIFO not full |
| 11..15 | - | never |

### WAIT sources

| src | syntax | waits until |
|---|---|---|
| 0 | `wait p gpio n` | absolute pin `n` == p |
| 1 | `wait p pin n` | pin `IN_BASE + n` == p |
| 2 | `wait p flag n` | flag `n` == p; **auto-clears** the flag when `p` = 1 |
| 3 | `wait 1 match` / `wait 0 match` | `(pins[IN_BASE..+15] & Y) == X` (or `!=`) |

`wait match` is Polyglot-specific: it waits for an arbitrary **multi-pin
pattern** in one instruction, for example USB SE0, an I2C bus-idle condition, or a
parallel-bus strobe with a specific address.

### IN sources / OUT destinations

| code | IN source | OUT destination |
|---|---|---|
| 0 | `pins` (from `IN_BASE`) | `pins` (from `OUT_BASE`, `OUT_COUNT` pins) |
| 1 | `x` | `x` |
| 2 | `y` | `y` |
| 3 | `null` (zeros) | `null` (discard) |
| 4 | `isr` | `pindirs` |
| 5 | `osr` | `pc` (computed jump) |
| 6 | `crc` | `isr` (sets ISR count = n) |
| 7 | `time` | - |

`IN`: shift `n` bits into ISR (direction from `IN_DIR`). With autopush, when
the count reaches `PUSH_THRESH` the ISR is pushed in the same instruction
(stall if RX is full). Pushed data is **right-justified**, so an 8-bit UART
byte arrives as `0x00XX` whichever way you shift.

`OUT`: shift `n` bits out of OSR. With autopull, an empty OSR is refilled from
the TX FIFO *in the same instruction*, so streams never lose a bit time.

### PUSH / PULL

`push [iffull] [block|noblock]`, `pull [ifempty] [block|noblock]` (default
`block`). Non-blocking pull from an empty FIFO copies `X` into OSR.
Non-blocking push into a full FIFO drops the data.

### MOV

Destinations: `pins`, `x`, `y`, `crc`, `pindirs`, `pc`, `isr`, `osr`.
Sources: `pins`, `x`, `y`, `null`, `crc`, `time`, `isr`, `osr`, `status`
(`{flags[7:0], 2'b0, rx_level[2:0], tx_level[2:0]}`).
Operations (prefix on the source): `!`/`~` invert, `::` bit-reverse, `><` byte-swap.

`mov pins, !pins` with `IN_BASE = OUT_BASE` toggles a set of pins in one
instruction. The USB program uses this for NRZI encoding.

### SET

Destinations: `pins`, `x`, `y`, `crc` (zero-extended), `pindirs`,
`xh` / `yh` (load the high byte, keep the low byte). This gives 16-bit
constants in two instructions.

### CRC engine

Each core has a 16-bit CRC register with a programmable polynomial
(`POLY` register), MSB-first or reflected (LSB-first) mode, and can be fed
automatically by every **1-bit** `OUT` and/or `IN` shift (`CRC_SRC`). It
computes CRC-16/MODBUS, CRC-16/USB, CRC-CCITT, CRC-8 (SMBus PEC, 1-Wire) and
CRC-5/USB at line rate for free. Read and initialise it with `mov`/`set`,
or read it from the host.

## Assembler syntax (`pgasm`)

```
.program name
.define  NAME value
.clkdiv  N                    ; divider
.out     base count           ; OUT pins    (base may be a pin name: uo0, uio3, ui1)
.set     base count           ; SET pins
.in      base                 ; IN pins
.side    base count [pindirs] ; side-set pins
.jmp_pin pin
.opendrain
.shift   out|in left|right [autopull|autopush] [threshold]
.crc     poly [reflect] [in] [out]
.wrap_target / .wrap          ; free loop
label:   instr [args] [side v] [delay]
```

`python -m polyglot.asm prog.pga` prints the listing.
`python -m polyglot.run prog.pga --push ... --vcd out.vcd` runs it on the
cycle-accurate model.
