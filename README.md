## Problem Statement

Given a large set of pins on a device determine which are
JTAG lines.

## Hardware Requirements

- Raspberry Pi Pico or RP2040-Zero (Generic, HW-368B)

- Couple of 33 to 330 ohms resistors - higher values are safer

## Cost

199 INR for RP2040-Zero HW-368B board and 10 INR (let's say) for the resistors.

## Safety Notes

Note: Use 33 to 330 ohms series resistors on all lines for safety.

[Optional for 3.3V device] Use a `TXS0108E` module board for voltage level
conversion. Pico is 3.3V ONLY!

![Level Converter](./TXS0108E-Module-1.jpg)

Note: By adding 330 ohms series resistors (and level conversion - if needed),
we become safe in almost all scenarios - can even short 3.3V high GPIO to GND
"accidentally".

## Usage

```bash
$ pyserial-miniterm /dev/ttyACM0
...

> s
================================
Starting scan for pattern:0110011101001101101000010111001001
FOUND! ntrst:GP0 tck:GP18 tms:GP8 tdo:GP2 tdi:GP3 IRlen:34
FOUND! ntrst:GP0 tck:GP18 tms:GP19 tdo:GP17 tdi:GP16 IRlen:8
FOUND! ntrst:GP1 tck:GP18 tms:GP4 tdo:GP2 tdi:GP3 IRlen:68
FOUND! ntrst:GP1 tck:GP18 tms:GP19 tdo:GP17 tdi:GP16 IRlen:8
FOUND! ntrst:GP2 tck:GP18 tms:GP19 tdo:GP17 tdi:GP16 IRlen:8
FOUND! ntrst:GP3 tck:GP18 tms:GP19 tdo:GP17 tdi:GP16 IRlen:8
FOUND! ntrst:GP4 tck:GP18 tms:GP19 tdo:GP17 tdi:GP16 IRlen:8
FOUND! ntrst:GP5 tck:GP18 tms:GP19 tdo:GP17 tdi:GP16 IRlen:8
FOUND! ntrst:GP10 tck:GP18 tms:GP19 tdo:GP17 tdi:GP16 IRlen:8
```

```bash
user@zion:~/repos/compare/JTAGenum$ pyserial-miniterm -e /dev/ttyACM0
--- Miniterm on /dev/ttyACM0  9600,8,N,1 ---
--- Quit: Ctrl+] | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H ---
...
> kk
Enter pin indices to scan (e.g. 18 19 17 16 0 1 2), end with '.' or newline:
15 Added GP15 (index 15)
16 Added GP16 (index 16)
17 Added GP17 (index 17)
18 Added GP18 (index 18)
19 Added GP19 (index 19)
20 Added GP20 (index 20)

Selected 6 pins for scanning.

> ii
================================
Starting scan for IDCODE...
(assumes IDCODE default DR)
ntrst:GP15 tck:GP18 tms:GP19 tdo:GP17 tdi:GP16  devices: 1
  0x04014C35 (anlogic eagle d20 EG4D20EG176 (IR:8))
ntrst:GP15 tck:GP18 tms:GP19 tdo:GP17 tdi:GP20  devices: 1
  0x04014C35 (anlogic eagle d20 EG4D20EG176 (IR:8))
ntrst:GP16 tck:GP18 tms:GP19 tdo:GP17 tdi:GP15  devices: 1
  0x04014C35 (anlogic eagle d20 EG4D20EG176 (IR:8))
ntrst:GP16 tck:GP18 tms:GP19 tdo:GP17 tdi:GP20  devices: 1
  0x04014C35 (anlogic eagle d20 EG4D20EG176 (IR:8))
ntrst:GP20 tck:GP18 tms:GP19 tdo:GP17 tdi:GP15  devices: 1
  0x04014C35 (anlogic eagle d20 EG4D20EG176 (IR:8))
ntrst:GP20 tck:GP18 tms:GP19 tdo:GP17 tdi:GP16  devices: 1
  0x04014C35 (anlogic eagle d20 EG4D20EG176 (IR:8))
```

Full exhaustive scans (without `Active Pin Selection` feature selected) can
take a while:

```bash
user@zion:~/repos/compare/JTAGenum$ pyserial-miniterm -e /dev/ttyACM0
--- Miniterm on /dev/ttyACM0  9600,8,N,1 ---
--- Quit: Ctrl+] | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H ---
ss
================================
Starting scan for pattern:0110011101001101101000010111001001
FOUND! ntrst:GP0 tck:GP1 tms:GP8 tdo:GP2 tdi:GP3 IRlen:68
FOUND! ntrst:GP0 tck:GP3 tms:GP5 tdo:GP2 tdi:GP1 IRlen:68
FOUND! ntrst:GP2 tck:GP18 tms:GP19 tdo:GP17 tdi:GP16 IRlen:8
FOUND! ntrst:GP5 tck:GP16 tms:GP9 tdo:GP2 tdi:GP3 IRlen:68
FOUND! ntrst:GP6 tck:GP12 tms:GP0 tdo:GP2 tdi:GP3 IRlen:34
FOUND! ntrst:GP6 tck:GP20 tms:GP14 tdo:GP2 tdi:GP3 IRlen:68
FOUND! ntrst:GP8 tck:GP5 tms:GP15 tdo:GP2 tdi:GP3 IRlen:34
FOUND! ntrst:GP8 tck:GP18 tms:GP19 tdo:GP17 tdi:GP16 IRlen:8
FOUND! ntrst:GP9 tck:GP13 tms:GP5 tdo:GP2 tdi:GP3 IRlen:68
FOUND! ntrst:GP9 tck:GP15 tms:GP10 tdo:GP2 tdi:GP3 IRlen:34
FOUND! ntrst:GP10 tck:GP13 tms:GP8 tdo:GP2 tdi:GP3 IRlen:34
FOUND! ntrst:GP11 tck:GP10 tms:GP13 tdo:GP2 tdi:GP3 IRlen:68
FOUND! ntrst:GP13 tck:GP5 tms:GP10 tdo:GP2 tdi:GP3 IRlen:68
FOUND! ntrst:GP14 tck:GP0 tms:GP8 tdo:GP2 tdi:GP3 IRlen:68
FOUND! ntrst:GP14 tck:GP3 tms:GP20 tdo:GP2 tdi:GP1 IRlen:34
FOUND! ntrst:GP14 tck:GP9 tms:GP20 tdo:GP2 tdi:GP3 IRlen:68
FOUND! ntrst:GP15 tck:GP3 tms:GP6 tdo:GP2 tdi:GP1 IRlen:34
FOUND! ntrst:GP15 tck:GP7 tms:GP9 tdo:GP2 tdi:GP3 IRlen:34
FOUND! ntrst:GP15 tck:GP16 tms:GP8 tdo:GP2 tdi:GP3 IRlen:68
FOUND! ntrst:GP20 tck:GP15 tms:GP18 tdo:GP2 tdi:GP3 IRlen:68
================================
```

Note: The onboard LED will blink rapidly during scans.

## Related / References

- https://github.com/cyphunk/JTAGenum

- https://rfcorner.in/posts/reversing-huidu-fpga-board/

- https://github.com/trabucayre/openFPGALoader (IDCODE stuff)

- https://robu.in/product/txs0108e-high-speed-full-duplex-8-channel-logic-level-converter/

- https://github.com/phdussud/pico-dirtyJtag

- https://github.com/kholia/xvc-pico
