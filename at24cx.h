/*
 at24cx - minimal driver for 16-bit addressed I2C EEPROMs (AT24C32 .. AT24C512)
 for the RP2040 / pico-sdk.
 
 Features:
   - arbitrary address / length reads (sequential read)
   - arbitrary address / length writes (automatically split on page boundaries)
   - write completion by ACK polling (no fixed delays)
   - every bus access has a timeout, nothing can hang forever
*/

#ifndef AT24CX_H
#define AT24CX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "hardware/i2c.h"

typedef enum {
    AT24CX_OK              =  0,
    AT24CX_ERR_I2C         = -1,  // bus error / NACK during a transfer
    AT24CX_ERR_NO_DEVICE   = -2,  // nothing answers on the configured address
    AT24CX_ERR_RANGE       = -3,  // addr + len is outside of the memory
    AT24CX_ERR_PARAM       = -4,  // bad argument / unsupported chip
    AT24CX_ERR_TIMEOUT     = -5,  // write cycle did not finish in time
} at24cx_err_t;

// Chip type. The value is the capacity in kbit, i.e. AT24C32 == 32 kbit == 4 KiB.
typedef enum {
    AT24CX_C32  = 32,
    AT24CX_C64  = 64,
    AT24CX_C128 = 128,
    AT24CX_C256 = 256,
    AT24CX_C512 = 512,
} at24cx_chip_t;

#define AT24CX_MAX_PAGE_SIZE    128u
#define AT24CX_DEFAULT_ADDR    0x57u   // A2=A1=A0=1, typical for DS3231 modules

typedef struct {
    i2c_inst_t   *i2c;        // i2c0 / i2c1
    uint8_t       addr;       // 7-bit address, 0x50..0x57
    at24cx_chip_t chip;
    bool          init_hw;    // true: at24cx_init() also sets up i2c + pins
    uint          sda_pin;    // used only when init_hw is true
    uint          scl_pin;
    uint          baudrate;   // Hz, used only when init_hw is true
} at24cx_config_t;

typedef struct {
    i2c_inst_t   *i2c;
    uint8_t       addr;
    at24cx_chip_t chip;
    uint32_t      size;       // capacity in bytes
    uint16_t      page_size;  // page write size in bytes
    uint32_t      write_timeout_us;
} at24cx_t;

at24cx_err_t at24cx_init(at24cx_t *dev, const at24cx_config_t *cfg);
at24cx_err_t at24cx_set_chip(at24cx_t *dev, uint8_t addr, at24cx_chip_t chip);
bool at24cx_bus_probe(i2c_inst_t *i2c, uint8_t addr);
at24cx_err_t at24cx_probe(const at24cx_t *dev);
at24cx_err_t at24cx_read(const at24cx_t *dev, uint32_t addr, void *buf, size_t len);
at24cx_err_t at24cx_write(const at24cx_t *dev, uint32_t addr, const void *buf, size_t len);
at24cx_err_t at24cx_read_byte(const at24cx_t *dev, uint32_t addr, uint8_t *value);
at24cx_err_t at24cx_write_byte(const at24cx_t *dev, uint32_t addr, uint8_t value);
at24cx_err_t at24cx_wait_ready(const at24cx_t *dev);
const char *at24cx_strerror(at24cx_err_t err);

#endif
