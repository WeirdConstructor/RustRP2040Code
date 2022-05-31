Bluetooth Raspberry Pi Pico RGB WS2812 LED Controller
=====================================================

Connection schematics, which pins to connect:

HC-05 Pins:

- 1 STATE
- 2 RXD
- 3 TXD
- 4 GND
- 5 VCC
- 6 EN

| Raspberry Pico    | HC-05 Module | WS2812 |
|-------------------|--------------|--------|
| 39 VSYS (+5V)     | 5 VCC        |        |
| 40 GND  (+5V)     | 4 GND        | 1 GND  |
| 19 GP14           | 1 STATE      |        |
| 21 GP16 UART0 TX  | 2 RXD        |        |
| 22 GP17 UART0 RX  | 3 TXD        |        |
| 24 GP18           |              | 2 DATA |
|                   |              |        |
