/*
 eeio protocol - EEPROM access over a packet link (USB bulk, 64 B packets).

  All integers are little endian.
 
  host -> device command packet (12 bytes):
      u8 opcode, u8[3] reserved, u32 addr, u32 len
 
  device -> host:
    INFO  : one packet  <BBHIIBxH>  status, proto_version, page_size, size,
            max_block, i2c_addr, chip_kbit  (fields are valid even when status
            is NODEV: they describe the *configured* chip)
    SETUP : addr = 7-bit I2C address, len = chip size in kbit (32..512);
            status packet. Takes effect immediately, not persistent.
    SCAN  : packet <B3x16s>: status + 128-bit bitmap of I2C addresses that ACK
            (bit n of byte n/8 == address n)
    READ  : status packet <B3xI> (status, len | detail); if status == OK the
            `len` data bytes follow in 64 B packets
    WRITE : status packet <B3xI>; if OK the host sends `len` data bytes in
            64 B packets, the device answers with a final status packet
            after the data has been written to the chip
 
    `len` of one READ/WRITE command is limited to max_block (see INFO); the host
    splits larger transfers into blocks.
*/

#ifndef EEIO_PROTO_H
#define EEIO_PROTO_H

#include <stdbool.h>
#include <stdint.h>
#include "at24cx.h"

#define EEIO_PROTO_VERSION        2
#define EEIO_PKT_SIZE            64
#define EEIO_MAX_BLOCK         4096
#define EEIO_DATA_TIMEOUT_MS   1000  // max gap between packets inside a command

enum {
    EEIO_CMD_INFO  = 0x01,
    EEIO_CMD_READ  = 0x02,
    EEIO_CMD_WRITE = 0x03,
    EEIO_CMD_SETUP = 0x04,
    EEIO_CMD_SCAN  = 0x05,
};

enum {
    EEIO_OK         = 0,
    EEIO_E_CMD      = 1,   // unknown command / malformed packet
    EEIO_E_RANGE    = 2,   // addr/len outside of the chip or > max_block
    EEIO_E_NODEV    = 3,   // EEPROM not detected
    EEIO_E_IO       = 4,   // I2C error, detail = at24cx_err_t
    EEIO_E_PARAM    = 5,   // SETUP: unsupported chip / address
};

typedef enum {
    EEIO_LINK_OK = 0,
    EEIO_LINK_TIMEOUT,
    EEIO_LINK_LOST,        // link went away (USB reset / unconfigured)
} eeio_link_result_t;

// Transport abstraction, implemented by main.c (USB) and by the host simulator
typedef struct {
    // send one packet (<= EEIO_PKT_SIZE bytes)
    eeio_link_result_t (*send)(void *ctx, const uint8_t *buf, uint16_t len, uint32_t timeout_ms);
    // receive one packet; timeout_ms == UINT32_MAX waits forever
    eeio_link_result_t (*recv)(void *ctx, uint8_t *buf, uint16_t *len, uint32_t timeout_ms);
    void *ctx;
} eeio_link_t;

// Serve commands until the link is lost.
void eeio_serve(const eeio_link_t *link, at24cx_t *dev);

#endif
