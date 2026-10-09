# pico-prog-eeprom

USB programmer for AT24Cxx I2C EEPROMs (AT24C32 ... AT24C512) built on an RP2040 / Raspberry Pi Pico.
The Pico enumerates as a vendor-specific USB device; the `host/eeio.py` tool reads, writes and verifies the chip.

## Layout

| File | Purpose |
|------|---------|
| `at24cx.c/.h` | EEPROM driver (pico-sdk I2C). Arbitrary address/length read and write, page-boundary handling, ACK polling, timeouts on every bus access. |
| `eeio_proto.c/.h` | Command protocol (INFO / READ / WRITE) on top of an abstract packet link. No USB or hardware dependencies. |
| `main.c`, `usb_common.h`, `dev_lowlevel.h` | Low-level USB device stack (from pico-examples) and the glue that connects it to `eeio_serve()`. |
| `host/eeio.py` | Host tool (Python 3, pyusb). |

## Wiring

Defaults (override with `-DEE_PIN_SDA=...` etc. in CMake, see top of `main.c`):

| Signal | Pico |
|--------|------|
| SDA | GP2 (I2C1 SDA) |
| SCL | GP3 (I2C1 SCL) |
| VCC / GND | 3V3(OUT) / GND |

Firmware defaults: I2C address `0x57`, chip `AT24CX_C256` (`EE_ADDRESS` / `EE_CHIP` in `main.c`).
Both can also be changed at runtime from the host with `--i2c-addr` / `--chip` (not persistent), so there is
no need to reflash when you switch chips. Many AT24C256 breakout boards have A0-A2 tied low, i.e. address `0x50`;
DS3231 boards with an AT24C32 use `0x57`.
The internal pull-ups are enabled, but external 4.7 kΩ pull-ups are recommended at 400 kHz.
UART0 (GP0/GP1) is free for debug output.

## Build

```sh
export PICO_SDK_PATH=/path/to/pico-sdk
mkdir build && cd build
cmake ..              # add -DEEIO_DEBUG=1 for USB/protocol debug prints on UART0
make
```

Flash `build/eeio.uf2` (hold BOOTSEL while plugging in the Pico, copy the file).

## Host tool

```sh
pip3 install pyusb      # needs libusb; run as root or add a udev rule for 0000:0001

host/eeio.py info                          # chip, size, page size, detection status
host/eeio.py scan                          # list every I2C address that answers
host/eeio.py --i2c-addr 0x50 --chip 256 info   # select address / chip type (kbit) for this run
host/eeio.py read   dump.bin               # whole chip
host/eeio.py read   dump.bin --addr 0x100 --len 64
host/eeio.py read   -                      # raw bytes to stdout
host/eeio.py write  image.bin              # writes, then verifies (use --no-verify to skip)
host/eeio.py write  image.bin --addr 0x200
host/eeio.py verify image.bin [--addr N]   # compare chip with a file
host/eeio.py reset                         # USB reset if a session got stuck
```

Numbers accept `0x` hex. Exit status: `0` ok, `1` error, `2` verify mismatch
(the first 16 differing addresses are printed). Use `--vid` / `--pid` if you change the USB ids.

## Protocol

Little endian, 64-byte bulk packets: EP1 OUT (host to device), EP2 IN (device to host).

* Command (12 B): `u8 opcode, u8[3] 0, u32 addr, u32 len` with `INFO=1`, `READ=2`, `WRITE=3`.
* `INFO` reply: `<BBHIIBxH>` status, protocol version, page size, chip size, max block (4096), I2C address,
  chip type in kbit. The geometry fields are filled even when the chip is not detected.
* `SETUP` (`addr` = I2C address, `len` = chip kbit) changes address/chip type; `SCAN` returns a 128-bit bitmap
  of I2C addresses that ACK.
* `READ`: status packet `<B3xI>`, then `len` data bytes. The device reads the whole block from the chip
  *before* answering, so an I2C error is reported up front instead of mid-stream.
* `WRITE`: status packet (request accepted), host sends `len` data bytes, device writes them and sends a
  final status packet.
* Status codes: `0` ok, `1` bad command, `2` range, `3` EEPROM not detected, `4` I2C error (detail = driver error),
  `5` unsupported chip/address in `SETUP`.
* Each block is at most 4096 bytes; the host splits larger transfers. If the host disappears mid-command
  the device drops it after 1 s and the next `eeio.py` run resynchronises automatically.

All I2C work happens in the main loop. The USB interrupt only hands packets over, so slow EEPROM write
cycles never block USB handling.

## Troubleshooting: "EEPROM not detected"

`eeio.py info` scans the bus and says what it found:

* it lists a device at `0x50`..`0x57` other than the configured one: use `--i2c-addr` (or change `EE_ADDRESS`);
* nothing answers: check SDA/SCL (GP2/GP3 by default, not swapped), GND, 3.3 V and pull-ups, and that the
  firmware pins in `main.c` match your wiring.

Detection only checks that the chip ACKs its address, so a wrong `--chip` does not cause it; it only gives the
wrong size and page size.

## Known limitations

* AT24C01..C16 (1-byte address) and AT24C1024 (17-bit address) are not supported.
* The USB vendor id `0x0000` is a placeholder; use your own or a pid.codes id for anything beyond testing.
