#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "at24cx.h"

#define READ_CHUNK                  256u // bytes per I2C read transaction
#define XFER_TIMEOUT_US          100000u // per I2C transaction (256 B @ 100k = 25 ms)
#define PROBE_TIMEOUT_US           5000u
#define DEFAULT_WRITE_TIMEOUT_US  20000u // datasheet tWR is 5 ms (10 ms older parts)

static at24cx_err_t chip_geometry(at24cx_chip_t chip, uint32_t *size, uint16_t *page) {
    switch (chip) {
    case AT24CX_C32:  *size = 4096;  *page = 32;  return AT24CX_OK;
    case AT24CX_C64:  *size = 8192;  *page = 32;  return AT24CX_OK;
    case AT24CX_C128: *size = 16384; *page = 64;  return AT24CX_OK;
    case AT24CX_C256: *size = 32768; *page = 64;  return AT24CX_OK;
    case AT24CX_C512: *size = 65536; *page = 128; return AT24CX_OK;
    default:          return AT24CX_ERR_PARAM;
    }
}

static at24cx_err_t check_range(const at24cx_t *dev, uint32_t addr, size_t len) {
    if (addr > dev->size || len > dev->size - addr)
        return AT24CX_ERR_RANGE;
    return AT24CX_OK;
}

at24cx_err_t at24cx_set_chip(at24cx_t *dev, uint8_t addr, at24cx_chip_t chip) {
    uint32_t size;
    uint16_t page;

    if ((addr & 0x78) != 0x50) // 0x50..0x57 only
        return AT24CX_ERR_PARAM;
    at24cx_err_t err = chip_geometry(chip, &size, &page);
    if (err != AT24CX_OK)
        return err;

    dev->addr = addr;
    dev->chip = chip;
    dev->size = size;
    dev->page_size = page;
    return AT24CX_OK;
}

at24cx_err_t at24cx_init(at24cx_t *dev, const at24cx_config_t *cfg) {
    if (!dev || !cfg || !cfg->i2c)
        return AT24CX_ERR_PARAM;

    memset(dev, 0, sizeof(*dev));
    at24cx_err_t err = at24cx_set_chip(dev, cfg->addr, cfg->chip);
    if (err != AT24CX_OK)
        return err;

    dev->i2c = cfg->i2c;
    dev->write_timeout_us = DEFAULT_WRITE_TIMEOUT_US;

    if (cfg->init_hw) {
        i2c_init(cfg->i2c, cfg->baudrate);
        gpio_set_function(cfg->sda_pin, GPIO_FUNC_I2C);
        gpio_set_function(cfg->scl_pin, GPIO_FUNC_I2C);
        gpio_pull_up(cfg->sda_pin);
        gpio_pull_up(cfg->scl_pin);
    }
    return at24cx_probe(dev);
}

bool at24cx_bus_probe(i2c_inst_t *i2c, uint8_t addr) {
    // A 1-byte read is the same trick the SDK bus-scan example uses
    // (the SDK does not allow zero-length transfers). It does not modify
    // the memory, it only moves the internal address pointer.
    uint8_t dummy;
    return i2c_read_timeout_us(i2c, addr, &dummy, 1, false, PROBE_TIMEOUT_US) == 1;
}

at24cx_err_t at24cx_probe(const at24cx_t *dev) {
    return at24cx_bus_probe(dev->i2c, dev->addr) ? AT24CX_OK : AT24CX_ERR_NO_DEVICE;
}

at24cx_err_t at24cx_wait_ready(const at24cx_t *dev) {
    // While the internal write cycle runs the chip NACKs its address.
    absolute_time_t deadline = make_timeout_time_us(dev->write_timeout_us);
    do {
        if (at24cx_probe(dev) == AT24CX_OK)
            return AT24CX_OK;
    } while (!time_reached(deadline));
    return AT24CX_ERR_TIMEOUT;
}

at24cx_err_t at24cx_read(const at24cx_t *dev, uint32_t addr, void *buf, size_t len) {
    at24cx_err_t err = check_range(dev, addr, len);
    if (err != AT24CX_OK)
        return err;

    uint8_t *out = buf;
    while (len) {
        size_t n = len < READ_CHUNK ? len : READ_CHUNK;
        uint8_t ptr[2] = { addr >> 8, addr & 0xFF };

        // set address pointer (dummy write), repeated start, sequential read
        if (i2c_write_timeout_us(dev->i2c, dev->addr, ptr, 2, true, XFER_TIMEOUT_US) != 2)
            return AT24CX_ERR_I2C;
        if (i2c_read_timeout_us(dev->i2c, dev->addr, out, n, false, XFER_TIMEOUT_US) != (int)n)
            return AT24CX_ERR_I2C;

        out += n;
        addr += n;
        len -= n;
    }
    return AT24CX_OK;
}

at24cx_err_t at24cx_write(const at24cx_t *dev, uint32_t addr, const void *buf, size_t len) {
    at24cx_err_t err = check_range(dev, addr, len);
    if (err != AT24CX_OK)
        return err;

    const uint8_t *in = buf;
    uint8_t frame[2 + AT24CX_MAX_PAGE_SIZE];

    while (len) {
        // never cross a page boundary inside a single write transaction,
        // the chip would wrap around inside the page and corrupt data
        size_t room = dev->page_size - (addr % dev->page_size);
        size_t n = len < room ? len : room;

        frame[0] = addr >> 8;
        frame[1] = addr & 0xFF;
        memcpy(&frame[2], in, n);

        if (i2c_write_timeout_us(dev->i2c, dev->addr, frame, 2 + n, false, XFER_TIMEOUT_US) != (int)(2 + n))
            return AT24CX_ERR_I2C;
        err = at24cx_wait_ready(dev);
        if (err != AT24CX_OK)
            return err;

        in += n;
        addr += n;
        len -= n;
    }
    return AT24CX_OK;
}

at24cx_err_t at24cx_read_byte(const at24cx_t *dev, uint32_t addr, uint8_t *value) {
    return at24cx_read(dev, addr, value, 1);
}

at24cx_err_t at24cx_write_byte(const at24cx_t *dev, uint32_t addr, uint8_t value) {
    return at24cx_write(dev, addr, &value, 1);
}

const char *at24cx_strerror(at24cx_err_t err) {
    switch (err) {
    case AT24CX_OK:            return "ok";
    case AT24CX_ERR_I2C:       return "I2C transfer failed";
    case AT24CX_ERR_NO_DEVICE: return "EEPROM not detected";
    case AT24CX_ERR_RANGE:     return "address out of range";
    case AT24CX_ERR_PARAM:     return "invalid parameter";
    case AT24CX_ERR_TIMEOUT:   return "write cycle timeout";
    default:                   return "unknown error";
    }
}
