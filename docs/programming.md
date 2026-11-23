# Teaching Polyglot a new protocol

The protocols in `programs/` are **examples, not features**. Nothing in the
silicon knows what UART, I2C or USB is. This page walks through adding a
protocol after the chip has been made, using only the toolchain.

## 1. Draw the waveform and count ticks

Choose a tick (the core clock after `CLKDIV`) so every interesting edge
lands on a whole number of ticks. Every instruction takes exactly
`1 + delay` ticks, so a waveform *is* a list of instructions.

WS2812 bit, 1.2 us = 12 ticks at 10 MHz (`clkdiv 6` at 60 MHz):

```
'1':  ‾‾‾‾‾‾‾‾____      high 8, low 4
'0':  ‾‾‾‾________      high 4, low 8
       |   |   |
tick   0   4   8   12
```

## 2. Write it

```asm
.program ws2812
.clkdiv 6
.side   uo1 1               ; the data pin is driven by side-set
.shift  out left autopull 16
.wrap_target
bit:
    out x, 1        side 0 [3]   ; ticks 8-11 of the previous bit: low
    jmp !x zero     side 1 [3]   ; ticks 0-3: high
    jmp bit         side 1 [3]   ; '1': ticks 4-7 high
zero:
    nop             side 0 [3]   ; '0': ticks 4-7 low
.wrap
```

Four instructions. `.wrap` closes the loop for free, autopull refills the
shift register without losing a tick, and when the host stops sending the
`out` stalls with the line low, which is exactly the WS2812 latch/reset
condition.

## 3. Simulate without hardware

```sh
python -m polyglot.run ws2812.pga --push 0xFF00,0x0F5A --cycles 2000 --vcd ws.vcd
```

The simulator is the same model the RTL is verified against cycle by cycle,
so what you see in `ws.vcd` is what the chip will do.

## 4. Run on the chip

```python
chip.load_json("ws2812.json", core=2)
chip.start(0b0100)
chip.push(2, 0xFF00, 0x0F5A)
```

## Idioms

| need | idiom |
|---|---|
| exact bit timing | delays on every instruction; make every branch path the same length |
| clock + data in one instruction | `out pins, 1 side 0 [1]` |
| sample in the middle of a bit | `wait 0 pin 0` on the start edge, then `set x, 7 [10]` |
| open-drain bus (I2C, 1-Wire, SMBus) | `.opendrain`: `set pins, 0` pulls low, `1` releases |
| clock stretching / handshakes | `wait 1 gpio SCL` |
| NRZI / toggle coding | `.in` = `.out` pins, then `mov pins, !pins` |
| bit stuffing | count ones in `y` with `jmp y--` |
| differential pairs | 2 side-set pins, write `0b01` / `0b10` |
| multi-pin pattern (SE0, bus idle) | `wait 1 match` with mask in `Y`, value in `X` |
| CRC / FCS | `.crc POLY reflect in`: free on every 1-bit shift |
| measure a pulse | count with `add x, 1` + `jmp pin`, or read `time` |
| byte framing to the host | `.shift in right autopush 8`: bytes arrive right-justified |
| coordinate cores | `flag set n` / `wait 1 flag n` (auto-clears) |
| several lanes in phase | start them with one `CTRL` write (EN + DIVSYNC) |

## Bigger examples in the repo

* `usb_ls_tx.pga`: USB low speed, written entirely in software: NRZI via
  `mov pins, !pins`, bit-stuffing via a `y` counter, and every branch
  balanced to exactly 40 ticks per bit. EOP generation and a completion flag.
* `manchester_tx.pga`: 10BASE-T style differential Manchester at 6 clocks
  per bit.
* `i2c_master.pga`: START, repeated START, STOP, ACK/NACK and clock
  stretching in 19 instructions, with a host-friendly 1-word-per-byte command format.
* The test suite writes a pulse-width meter *inline in a test* and runs it
  next to three other protocols.
