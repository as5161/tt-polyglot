# Host interface and register map

The host (the RP2350 on the Tiny Tapeout demo board) talks to Polyglot
over a **mode-0 SPI slave** on the dedicated inputs:

| pin | signal |
|---|---|
| `ui[7]` | `CS_n` |
| `ui[6]` | `SCK`  (at most clk / 8, e.g. 6 MHz at 50 MHz) |
| `ui[5]` | `MOSI` |
| `uo[7]` | `MISO` |

## Frame format

```
CS_n low
  16-bit header : W (1 = write), ADDR[14:0]
  24-bit word 0 : MSB first
  24-bit word 1 ...           (any number of words)
CS_n high
```

The address auto-increments after every word, **except** at the per-core
FIFO registers.  Each core owns a 128-word instruction bank (an IHP SRAM
macro): loading or patching one core never disturbs the others, and a bank
can only be accessed while its core is stopped. A burst to `FIFO` streams many words into or out of one FIFO.

Reads are side-effect free while a word is being shifted. A FIFO entry is
consumed only after the master has clocked out the **complete** 24-bit word
*and* that word was valid (bit 23). Aborting a frame never loses data.

## Map

| address | name | access | contents |
|---|---|---|---|
| `0x0400 + 0x100*c + n` | `IMEM[c][n]` | R/W | core `c`'s instruction bank, word `n` = 0..127 (24-bit instructions). Only while core `c` is stopped (`EN[c] = 0`): writes to a running core's bank are ignored and reads return 0 |
| `0x80 + 8c + 0` | `CLKDIV` | R/W | `[15:0]` divider (0, 1 = full speed) |
| `0x80 + 8c + 1` | `EXECCTRL` | R/W | `WRAP_BOT[6:0] WRAP_TOP[13:7] JMP_PIN[18:14] SIDE_PINDIR[19] OD[20]` |
| `0x80 + 8c + 2` | `SHIFTCTRL` | R/W | `OUT_DIR[0] IN_DIR[1] AUTOPULL[2] AUTOPUSH[3] PULL_TH[8:4] PUSH_TH[13:9] CRC_SRC[15:14] CRC_REFLECT[16]` (thresholds: 0 = 16; dir 0 = right/LSB first) |
| `0x80 + 8c + 3` | `PINCTRL` | R/W | `OUT_BASE[4:0] OUT_COUNT[9:5] SET_BASE[14:10] SET_COUNT[18:15]` |
| `0x80 + 8c + 4` | `PINCTRL2` | R/W | `IN_BASE[4:0] SIDE_BASE[9:5] SIDE_COUNT[11:10]` |
| `0x80 + 8c + 5` | `FIFO` | W: push TX / R: RX | read: `{valid, 7'b0, data[15:0]}`; full TX drops writes |
| `0x80 + 8c + 6` | `PC` | R/W | write = jump (also cancels pending delay) |
| `0x80 + 8c + 7` | `EXEC` / `STAT` | W / R | write: execute this word on core c *now* (even when the core is stopped); read: `{exec_pending[7], rx_level[5:3], tx_level[2:0]}` |
| `0xA0` | `CTRL` | R/W | `EN[3:0]`, write-1 pulses: `RESTART[7:4]` (clears ISR/OSR/delay), `DIVSYNC[11:8]` (phase-aligns dividers), `FIFOCLR[15:12]` |
| `0xA1` | `STEP` | W | `[3:0]`: give the selected cores exactly one tick |
| `0xA2` | `FLAGS` | R/W | `[7:0]` inter-core / host event flags |
| `0xA3` | `FIFOSTAT` | R | `TXFULL[3:0] TXEMPTY[7:4] RXFULL[11:8] RXEMPTY[15:12]` |
| `0xA4` | `PINS` | R | synchronised pin space `[23:0]` |
| `0xA5` | `ID` | R | `0x504702` ("PG", version 2) |
| `0xA6` | `TIME` | R | 16-bit cycle counter |
| `0xA8 + c` | `POLY` | R/W | CRC polynomial for core c |
| `0xAC + c` / `0xB0 + c` / `0xB4 + c` | `X` / `Y` / `CRC` | R | debug read-back |

`c` = core 0..3.  All registers except the IMEM banks live in page 0 (`ADDR[14:8] = 0`); other addresses read 0. `DIVSYNC` + `EN` in a single write starts several cores on
the same cycle, phase-locked (used for multi-lane protocols).

## Debugging on silicon

`EN = 0` + `STEP` + the `X`/`Y`/`CRC`/`PC`/`STAT` read-back registers make a
complete single-step debugger, and `EXEC` lets the host poke any instruction
(for example `set pindirs, 1`) into a stopped core.
