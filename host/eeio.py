"""
    Host tool for the pico-prog-eeprom programmer.

    sudo pip3 install pyusb

    eeio.py [--i2c-addr 0x50] [--chip 256] info
    eeio.py scan
    eeio.py read   dump.bin [--addr 0] [--len N]
    eeio.py write  image.bin [--addr 0] [--no-verify]
    eeio.py verify image.bin [--addr 0]
    eeio.py reset

    Exit status: 0 ok, 1 error, 2 verify mismatch.
"""

import argparse
import struct
import sys
import usb.core
import usb.util

USB_VENDOR_ID  = 0x0000
USB_PRODUCT_ID = 0x0001

PKT_SIZE       = 64
PROTO_VERSION  = 2

CMD_INFO  = 0x01
CMD_READ  = 0x02
CMD_WRITE = 0x03
CMD_SETUP = 0x04
CMD_SCAN  = 0x05

STATUS_TEXT = {
    0: "ok",
    1: "bad command",
    2: "address/length out of range",
    3: "EEPROM not detected",
    4: "I2C error",
    5: "unsupported chip type or I2C address",
}

CMD_TIMEOUT_MS   = 2000
DATA_TIMEOUT_MS  = 5000
WRITE_TIMEOUT_MS = 20000  # 4 KiB of 32 B pages: ~130 write cycles of <= 5 ms


class EeioError(Exception):
    pass


def is_timeout(exc):
    cls = getattr(usb.core, "USBTimeoutError", None)
    if cls is not None and isinstance(exc, cls):
        return True
    return getattr(exc, "errno", None) in (110, 60, 10060)


def progress(prefix, done, total, width=40):
    if not sys.stderr.isatty():
        return
    frac = 1.0 if total == 0 else done / float(total)
    filled = int(width * frac)
    sys.stderr.write("\r%s |%s%s| %5.1f%%" % (
        prefix, "█" * filled, "-" * (width - filled), 100.0 * frac))
    if done >= total:
        sys.stderr.write("\n")
    sys.stderr.flush()


class Eeio:

    def __init__(self, vid=USB_VENDOR_ID, pid=USB_PRODUCT_ID):
        self.dev = usb.core.find(idVendor=vid, idProduct=pid)
        if self.dev is None:
            raise EeioError("device %04x:%04x not found" % (vid, pid))
        try:
            cfg = self.dev.get_active_configuration()
        except usb.core.USBError:
            self.dev.set_configuration()
            cfg = self.dev.get_active_configuration()
        intf = cfg[(0, 0)]
        self.outep = usb.util.find_descriptor(intf, custom_match=lambda e:
            usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT)
        self.inep = usb.util.find_descriptor(intf, custom_match=lambda e:
            usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)
        if self.inep is None or self.outep is None:
            raise EeioError("bulk endpoints not found")
        self.page_size = 0
        self.size = 0
        self.max_block = 0
        self.i2c_addr = 0
        self.chip = 0
        self.detected = False

    def flush(self):
        """Throw away anything a previous, aborted session left in the pipe."""
        while True:
            try:
                self.inep.read(PKT_SIZE, timeout=100)
            except usb.core.USBError as e:
                if is_timeout(e):
                    return
                raise

    def _write(self, data, timeout):
        try:
            self.outep.write(data, timeout=timeout)
        except usb.core.USBError as e:
            raise EeioError("USB write failed: %s" % e)

    def _read(self, size, timeout):
        try:
            return bytes(self.inep.read(size, timeout=timeout))
        except usb.core.USBError as e:
            raise EeioError("USB read failed: %s" % e)

    def _command(self, op, addr=0, length=0, timeout=CMD_TIMEOUT_MS):
        self._write(struct.pack("<B3xII", op, addr, length), timeout)

    def _status(self, timeout):
        pkt = self._read(PKT_SIZE, timeout)
        if len(pkt) < 8:
            raise EeioError("short status packet (%d bytes)" % len(pkt))
        status, value = struct.unpack("<B3xI", pkt[:8])
        if status != 0:
            text = STATUS_TEXT.get(status, "status %d" % status)
            if status == 4:
                text += " (driver error %d)" % -struct.unpack("<i", pkt[4:8])[0]
            raise EeioError(text)
        return value

    def connect(self, i2c_addr=None, chip=None, retries=3):
        """Sync with the device, optionally select address/chip, fetch geometry."""
        last = None
        for _ in range(retries):
            self.flush()
            try:
                self.info()
                break
            except EeioError as e:
                last = e
        else:
            raise last
        if (i2c_addr is not None and i2c_addr != self.i2c_addr) or \
           (chip is not None and chip != self.chip):
            self.setup(self.i2c_addr if i2c_addr is None else i2c_addr,
                       self.chip if chip is None else chip)
            self.info()

    def info(self):
        """Fetch geometry. A missing EEPROM is not an error here (see require_chip)."""
        self._command(CMD_INFO)
        pkt = self._read(PKT_SIZE, CMD_TIMEOUT_MS)
        if len(pkt) < 16:
            raise EeioError("short info packet (old firmware?)")
        (status, version, self.page_size, self.size, self.max_block,
         self.i2c_addr, self.chip) = struct.unpack("<BBHIIBxH", pkt[:16])
        if version != PROTO_VERSION:
            raise EeioError("protocol version %d not supported (expected %d)"
                            % (version, PROTO_VERSION))
        if status not in (0, 3):
            raise EeioError(STATUS_TEXT.get(status, "status %d" % status))
        self.detected = status == 0
        return self.detected

    def setup(self, i2c_addr, chip):
        self._command(CMD_SETUP, i2c_addr, chip)
        self._status(CMD_TIMEOUT_MS)

    def scan(self):
        """Return the sorted list of 7-bit I2C addresses that answer."""
        self._command(CMD_SCAN)
        pkt = self._read(PKT_SIZE, CMD_TIMEOUT_MS)
        if len(pkt) < 20 or pkt[0] != 0:
            raise EeioError("bad scan reply")
        return [a for a in range(128) if pkt[4 + a // 8] & (1 << (a % 8))]

    def require_chip(self):
        if self.detected:
            return
        found = self.scan()
        msg = "EEPROM not detected at I2C address 0x%02x (configured as AT24C%d)." % (
            self.i2c_addr, self.chip)
        if found:
            msg += " Devices answering on the bus: %s." % ", ".join("0x%02x" % a for a in found)
            eeproms = [a for a in found if 0x50 <= a <= 0x57]
            if eeproms:
                msg += " Try --i2c-addr 0x%02x." % eeproms[0]
        else:
            msg += " Nothing answers on the bus - check wiring (SDA/SCL, GND, power, pull-ups)."
        raise EeioError(msg)

    def read(self, addr, length, show_progress=False):
        self.require_chip()
        self._check_range(addr, length)
        out = bytearray()
        while len(out) < length:
            n = min(self.max_block, length - len(out))
            self._command(CMD_READ, addr + len(out), n)
            self._status(DATA_TIMEOUT_MS)
            chunk = self._read(n, DATA_TIMEOUT_MS)
            if len(chunk) != n:
                raise EeioError("short read: got %d of %d bytes" % (len(chunk), n))
            out += chunk
            if show_progress:
                progress("read ", len(out), length)
        return bytes(out)

    def write(self, addr, data, show_progress=False):
        self.require_chip()
        self._check_range(addr, len(data))
        done = 0
        while done < len(data):
            n = min(self.max_block, len(data) - done)
            self._command(CMD_WRITE, addr + done, n)
            self._status(DATA_TIMEOUT_MS)           # request accepted
            self._write(data[done:done + n], DATA_TIMEOUT_MS)
            self._status(WRITE_TIMEOUT_MS)          # data is in the chip
            done += n
            if show_progress:
                progress("write", done, len(data))

    def verify(self, addr, data, show_progress=False):
        """Return a list of (address, expected, actual) mismatches."""
        actual = self.read(addr, len(data), show_progress)
        return [(addr + i, data[i], actual[i])
                for i in range(len(data)) if data[i] != actual[i]]

    def reset(self):
        self.dev.reset()

    def _check_range(self, addr, length):
        if addr < 0 or length < 0 or addr + length > self.size:
            raise EeioError("range 0x%x..0x%x is outside of the EEPROM (size 0x%x)"
                            % (addr, addr + length, self.size))


def report_mismatches(bad, limit=16):
    print("VERIFY FAILED: %d byte(s) differ" % len(bad))
    for a, exp, act in bad[:limit]:
        print("  0x%04x: expected 0x%02x, read 0x%02x" % (a, exp, act))
    if len(bad) > limit:
        print("  ... and %d more" % (len(bad) - limit))


def read_file(path):
    with open(path, "rb") as fh:
        data = fh.read()
    if not data:
        raise EeioError("%s is empty" % path)
    return data


def cmd_info(ee, args):
    print("chip type   : AT24C%d at I2C address 0x%02x" % (ee.chip, ee.i2c_addr))
    print("EEPROM size : %d bytes (%d KiB)" % (ee.size, ee.size // 1024))
    print("page size   : %d bytes" % ee.page_size)
    print("max block   : %d bytes" % ee.max_block)
    ee.require_chip()               # raises with a diagnosis if the chip is missing
    print("detected    : yes")
    return 0


def cmd_scan(ee, args):
    found = ee.scan()
    if not found:
        print("no I2C device answers")
        return 1
    for a in found:
        note = "  <- EEPROM range" if 0x50 <= a <= 0x57 else ""
        print("0x%02x%s" % (a, note))
    return 0


def cmd_read(ee, args):
    length = ee.size - args.addr if args.len is None else args.len
    data = ee.read(args.addr, length, show_progress=args.output != "-")
    if args.output == "-":
        sys.stdout.buffer.write(data)
    else:
        with open(args.output, "wb") as fh:
            fh.write(data)
        print("* read %d bytes from 0x%x -> %s" % (len(data), args.addr, args.output))
    return 0


def cmd_write(ee, args):
    data = read_file(args.input)
    ee.write(args.addr, data, show_progress=True)
    print("* wrote %d bytes at 0x%x" % (len(data), args.addr))
    if args.no_verify:
        return 0
    bad = ee.verify(args.addr, data, show_progress=True)
    if bad:
        report_mismatches(bad)
        return 2
    print("* verify OK")
    return 0


def cmd_verify(ee, args):
    data = read_file(args.input)
    bad = ee.verify(args.addr, data, show_progress=True)
    if bad:
        report_mismatches(bad)
        return 2
    print("* verify OK (%d bytes)" % len(data))
    return 0


def cmd_reset(ee, args):
    ee.reset()
    print("* USB reset sent")
    return 0


def number(text):
    return int(text, 0)     # accepts 123, 0x7b, 0o173, 0b1111011


def build_parser():
    p = argparse.ArgumentParser(description="Pico EEPROM programmer (AT24Cxx over I2C)")
    p.add_argument("--vid", type=number, default=USB_VENDOR_ID, help="USB vendor id")
    p.add_argument("--pid", type=number, default=USB_PRODUCT_ID, help="USB product id")
    p.add_argument("--i2c-addr", type=number, default=None,
                   help="EEPROM I2C address 0x50..0x57 (default: firmware setting)")
    p.add_argument("--chip", type=number, default=None, choices=(32, 64, 128, 256, 512),
                   help="chip type in kbit, e.g. 256 for AT24C256 (default: firmware setting)")
    sub = p.add_subparsers(dest="command", metavar="command")
    sub.required = True

    s = sub.add_parser("info", help="show EEPROM geometry")
    s.set_defaults(func=cmd_info)

    s = sub.add_parser("scan", help="list I2C devices that answer on the bus")
    s.set_defaults(func=cmd_scan)

    s = sub.add_parser("read", help="read EEPROM into a file ('-' = stdout)")
    s.add_argument("output")
    s.add_argument("--addr", type=number, default=0, help="start address (default 0)")
    s.add_argument("--len", type=number, default=None, help="bytes to read (default: up to the end)")
    s.set_defaults(func=cmd_read)

    s = sub.add_parser("write", help="write a file into the EEPROM (verifies afterwards)")
    s.add_argument("input")
    s.add_argument("--addr", type=number, default=0, help="start address (default 0)")
    s.add_argument("--no-verify", action="store_true", help="skip the read-back check")
    s.set_defaults(func=cmd_write)

    s = sub.add_parser("verify", help="compare EEPROM contents with a file")
    s.add_argument("input")
    s.add_argument("--addr", type=number, default=0, help="start address (default 0)")
    s.set_defaults(func=cmd_verify)

    s = sub.add_parser("reset", help="USB-reset the programmer (recover from a stuck transfer)")
    s.set_defaults(func=cmd_reset)
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        ee = Eeio(args.vid, args.pid)
        if args.command == "reset":
            return args.func(ee, args)
        ee.connect(args.i2c_addr, args.chip)
        return args.func(ee, args)
    except EeioError as e:
        print("error: %s" % e, file=sys.stderr)
        return 1
    except OSError as e:
        print("error: %s" % e, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
