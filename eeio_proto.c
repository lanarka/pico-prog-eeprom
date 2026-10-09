#include "eeio_proto.h"
#include <string.h>

static uint8_t block[EEIO_MAX_BLOCK];

static uint32_t get_u32(const uint8_t *p) {
    return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void put_u32(uint8_t *p, uint32_t v) {
    p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}

static eeio_link_result_t send_status(const eeio_link_t *l, uint8_t status, uint32_t value) {
    uint8_t pkt[8] = { status, 0, 0, 0 };
    put_u32(&pkt[4], value);
    return l->send(l->ctx, pkt, sizeof(pkt), EEIO_DATA_TIMEOUT_MS);
}

static eeio_link_result_t cmd_info(const eeio_link_t *l, const at24cx_t *dev) {
    uint8_t pkt[16] = { EEIO_OK, EEIO_PROTO_VERSION };

    if (at24cx_probe(dev) != AT24CX_OK)
        pkt[0] = EEIO_E_NODEV;
    pkt[2] = dev->page_size;
    pkt[3] = dev->page_size >> 8;
    put_u32(&pkt[4], dev->size);
    put_u32(&pkt[8], EEIO_MAX_BLOCK);
    pkt[12] = dev->addr;
    pkt[14] = dev->chip;
    pkt[15] = dev->chip >> 8;
    return l->send(l->ctx, pkt, sizeof(pkt), EEIO_DATA_TIMEOUT_MS);
}

static eeio_link_result_t cmd_setup(const eeio_link_t *l, at24cx_t *dev, uint32_t addr, uint32_t chip) {
    if (addr > 0x7F || chip > 0xFFFF ||
        at24cx_set_chip(dev, addr, (at24cx_chip_t)chip) != AT24CX_OK)
        return send_status(l, EEIO_E_PARAM, 0);
    return send_status(l, EEIO_OK, dev->size);
}

static eeio_link_result_t cmd_scan(const eeio_link_t *l, const at24cx_t *dev) {
    uint8_t pkt[20] = { EEIO_OK };
    for (uint8_t a = 0x08; a < 0x78; a++) // skip reserved addresses
        if (at24cx_bus_probe(dev->i2c, a))
            pkt[4 + a / 8] |= 1u << (a % 8);
    return l->send(l->ctx, pkt, sizeof(pkt), EEIO_DATA_TIMEOUT_MS);
}

static uint8_t check_request(const at24cx_t *dev, uint32_t addr, uint32_t len){
    if (len == 0 || len > EEIO_MAX_BLOCK)
        return EEIO_E_RANGE;
    if (addr > dev->size || len > dev->size - addr)
        return EEIO_E_RANGE;
    return EEIO_OK;
}

static eeio_link_result_t cmd_read(const eeio_link_t *l, const at24cx_t *dev, uint32_t addr, uint32_t len) {
    uint8_t st = check_request(dev, addr, len);
    if (st != EEIO_OK)
        return send_status(l, st, 0);//

    at24cx_err_t err = at24cx_read(dev, addr, block, len);
    if (err != AT24CX_OK)
        return send_status(l, EEIO_E_IO, (uint32_t)(-err));

    eeio_link_result_t r = send_status(l, EEIO_OK, len);
    for (uint32_t off = 0; r == EEIO_LINK_OK && off < len; off += EEIO_PKT_SIZE) {
        uint32_t n = len - off < EEIO_PKT_SIZE ? len - off : EEIO_PKT_SIZE;
        r = l->send(l->ctx, &block[off], n, EEIO_DATA_TIMEOUT_MS);
    }
    return r;
}

static eeio_link_result_t cmd_write(const eeio_link_t *l, const at24cx_t *dev, uint32_t addr, uint32_t len) {
    uint8_t st = check_request(dev, addr, len);
    if (st != EEIO_OK)
        return send_status(l, st, 0);//

    eeio_link_result_t r = send_status(l, EEIO_OK, len);
    if (r != EEIO_LINK_OK)
        return r;

    for (uint32_t off = 0; off < len; ) {
        uint32_t want = len - off < EEIO_PKT_SIZE ? len - off : EEIO_PKT_SIZE;
        uint8_t pkt[EEIO_PKT_SIZE];
        uint16_t got = 0;

        r = l->recv(l->ctx, pkt, &got, EEIO_DATA_TIMEOUT_MS);
        if (r != EEIO_LINK_OK)
            return r;                 // host vanished -> drop the command
        if (got != want)
            return EEIO_LINK_TIMEOUT; // protocol desync -> drop the command
        memcpy(&block[off], pkt, got);
        off += got;
    }

    at24cx_err_t err = at24cx_write(dev, addr, block, len);
    if (err != AT24CX_OK)
        return send_status(l, EEIO_E_IO, (uint32_t)(-err));
    return send_status(l, EEIO_OK, len);
}

void eeio_serve(const eeio_link_t *l, at24cx_t *dev) {
    for (;;) {
        uint8_t pkt[EEIO_PKT_SIZE];
        uint16_t len = 0;
        eeio_link_result_t r = l->recv(l->ctx, pkt, &len, UINT32_MAX);
        if (r == EEIO_LINK_LOST)
            return;
        if (r != EEIO_LINK_OK)
            continue;

        if (len != 12) {
            r = send_status(l, EEIO_E_CMD, 0);
        } else {
            uint32_t addr = get_u32(&pkt[4]);
            uint32_t n = get_u32(&pkt[8]);
            switch (pkt[0]) {
            case EEIO_CMD_INFO:  r = cmd_info(l, dev);              break;
            case EEIO_CMD_READ:  r = cmd_read(l, dev, addr, n);     break;
            case EEIO_CMD_WRITE: r = cmd_write(l, dev, addr, n);    break;
            case EEIO_CMD_SETUP: r = cmd_setup(l, dev, addr, n);    break;
            case EEIO_CMD_SCAN:  r = cmd_scan(l, dev);              break;
            default:             r = send_status(l, EEIO_E_CMD, 0); break;
            }
        }
        if (r == EEIO_LINK_LOST)
            return;
        // TIMEOUT: command abandoned, go back to waiting for a new one
    }
}
