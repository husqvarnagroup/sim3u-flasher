/*
 * SPDX-FileCopyrightText: GARDENA GmbH
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/* A SWD target on the far end of the wire, for running the flasher with no
 * hardware attached.
 *
 * This file replaces the GPIO layer in a GPIO_MOCK build: gpio_open hands
 * back two pins that lead here rather than to /dev/mem, and the bit-bang
 * accessors drive a SWD slave state machine.  swd.c, sim3u_flash.c and main.c
 * are the production files, unmodified -- exercising them is the whole point,
 * so the emulation sits strictly below them.
 *
 * Protocol only, no timing.  Bits the target samples arrive on the rising
 * edge of SWCLK; bits it drives are handed over when the host reads SWDIO,
 * which is where swd.c samples.  Nothing models setup or hold, so SWD_DELAY
 * has no effect and the wire is always electrically perfect.
 *
 * The target never answers WAIT or FAULT.  Those paths, and the retry budget
 * built for them, remain hardware-only. */

#include "gpio.h"
#include "swd.h"

#include <stdint.h>
#include <string.h>

#define PIN_CLK 0
#define PIN_DIO 1

/* Consecutive ones on SWDIO that the DP treats as a line reset */
#define LINE_RESET_ONES 50

/* Turnaround, three ack bits, a 32-bit word and its parity */
#define OUT_MAX 37

/* Turnaround, 32 data bits and parity, all clocked in after a write ack */
#define WDATA_BITS 34

#define TARGET_IDCODE 0x2BA01477u

enum target_state {
    STATE_UNSYNCED, /* nothing sensible seen yet */
    STATE_RESET, /* line reset seen, waiting for it to end */
    STATE_IDLE, /* between transfers */
    STATE_REQUEST, /* collecting the eight request bits */
    STATE_WDATA, /* collecting the data phase of a write */
};

static struct {
    int clk_high;
    int dio_level; /* level the host drives, meaningless while it reads */
    int dio_is_output;

    enum target_state state;
    int ones; /* consecutive ones, for line reset detection */

    uint8_t req;
    int req_bits;

    int w_apndp;
    uint8_t w_addr;
    uint64_t w_data;
    int w_bits;

    uint8_t out[OUT_MAX];
    int out_len;
    int out_pos;

    uint32_t ctrl_stat;
    uint32_t select;
    uint32_t csw;
} tgt;

static int parity32(uint32_t v)
{
    v ^= v >> 16;
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return (int)(v & 1u);
}

/*
 * Registers
 */

static uint32_t read_reg(int apndp, uint8_t addr)
{
    if (apndp) {
        return addr == AP_CSW ? tgt.csw : 0;
    }
    switch (addr) {
    case DP_IDCODE:
        return TARGET_IDCODE;
    case DP_CTRL:
        return tgt.ctrl_stat;
    default:
        return 0;
    }
}

static void write_reg(int apndp, uint8_t addr, uint32_t val)
{
    if (apndp) {
        if (addr == AP_CSW) {
            tgt.csw = val;
        }
        return;
    }
    switch (addr) {
    case DP_CTRL:
        /* Both domains power up the instant they are asked to */
        tgt.ctrl_stat = val;
        if (val & DP_CTRL_CDBGPWRUPREQ) {
            tgt.ctrl_stat |= DP_CTRL_CDBGPWRUPACK;
        }
        if (val & DP_CTRL_CSYSPWRUPREQ) {
            tgt.ctrl_stat |= DP_CTRL_CSYSPWRUPACK;
        }
        break;
    case DP_SELECT:
        tgt.select = val;
        break;
    default:
        break;
    }
}

/*
 * Bits the target drives
 */

static void queue_bit(int bit)
{
    if (tgt.out_len < OUT_MAX) {
        tgt.out[tgt.out_len++] = (uint8_t)bit;
    }
}

/* One turnaround bit the host discards, then the ack, least significant
   first.  Every request this target understands is answered OK. */
static void queue_ack(void)
{
    tgt.out_len = 0;
    tgt.out_pos = 0;
    queue_bit(0);
    for (int i = 0; i < 3; i++) {
        queue_bit((SWD_ACK_OK >> i) & 1);
    }
}

static void queue_word(uint32_t val)
{
    for (int i = 0; i < 32; i++) {
        queue_bit((int)((val >> i) & 1u));
    }
    queue_bit(parity32(val));
}

/*
 * Request decoding
 */

static void handle_request(void)
{
    uint8_t r = tgt.req;
    int apndp = (r >> 1) & 1;
    int rnw = (r >> 2) & 1;
    int a2 = (r >> 3) & 1;
    int a3 = (r >> 4) & 1;
    int par = (r >> 5) & 1;
    int stop = (r >> 6) & 1;
    int park = (r >> 7) & 1;
    uint8_t addr = (uint8_t)((a2 << 2) | (a3 << 3));

    tgt.req_bits = 0;
    tgt.state = STATE_IDLE;

    /* Framing or parity that does not add up is not a request: the switch
       sequence sent before a line reset lands here */
    if (stop != 0 || park != 1 || par != (apndp ^ rnw ^ a2 ^ a3)) {
        return;
    }

    queue_ack();

    if (rnw) {
        queue_word(read_reg(apndp, addr));
        return;
    }

    tgt.w_apndp = apndp;
    tgt.w_addr = addr;
    tgt.w_data = 0;
    tgt.w_bits = 0;
    tgt.state = STATE_WDATA;
}

static void sample_bit(int bit)
{
    tgt.ones = bit ? tgt.ones + 1 : 0;

    /* A line reset outranks whatever the target thought it was doing, which
       is what lets swd_connect recover the link from any state */
    if (tgt.ones >= LINE_RESET_ONES) {
        tgt.state = STATE_RESET;
        tgt.req_bits = 0;
        tgt.w_bits = 0;
        tgt.out_len = 0;
        tgt.out_pos = 0;
        tgt.select = 0;
        return;
    }

    switch (tgt.state) {
    case STATE_UNSYNCED:
        break;
    case STATE_RESET:
        if (!bit) {
            tgt.state = STATE_IDLE;
        }
        break;
    case STATE_IDLE:
        if (bit) {
            tgt.req = 1;
            tgt.req_bits = 1;
            tgt.state = STATE_REQUEST;
        }
        break;
    case STATE_REQUEST:
        tgt.req |= (uint8_t)(bit << tgt.req_bits);
        tgt.req_bits++;
        if (tgt.req_bits == 8) {
            handle_request();
        }
        break;
    case STATE_WDATA:
        tgt.w_data |= (uint64_t)bit << tgt.w_bits;
        tgt.w_bits++;
        if (tgt.w_bits == WDATA_BITS) {
            /* Bit 0 was the turnaround, the parity is not checked */
            write_reg(tgt.w_apndp, tgt.w_addr, (uint32_t)(tgt.w_data >> 1));
            tgt.state = STATE_IDLE;
        }
        break;
    }
}

/*
 * The wire
 */

static void clk_edge(int high)
{
    if (high && !tgt.clk_high && tgt.dio_is_output) {
        sample_bit(tgt.dio_level);
    }
    tgt.clk_high = high;
}

void gpio_set(const gpio_pin_t *p)
{
    gpio_put(p, 1);
}

void gpio_clr(const gpio_pin_t *p)
{
    gpio_put(p, 0);
}

void gpio_put(const gpio_pin_t *p, int high)
{
    if (p->bit == PIN_CLK) {
        clk_edge(high);
    } else {
        tgt.dio_level = high;
    }
}

void gpio_sync(const gpio_pin_t *p)
{
    (void)p;
}

int gpio_get(const gpio_pin_t *p)
{
    if (p->bit != PIN_DIO) {
        return tgt.clk_high;
    }
    if (tgt.out_pos < tgt.out_len) {
        return tgt.out[tgt.out_pos++];
    }
    return 1; /* nothing driving it, and the line is pulled up */
}

void gpio_dir(const gpio_pin_t *p, int output)
{
    if (p->bit == PIN_DIO) {
        tgt.dio_is_output = output;
    }
}

/*
 * Setup / teardown
 */

int gpio_open(gpio_t *g)
{
    memset(g, 0, sizeof(*g));
    memset(&tgt, 0, sizeof(tgt));

    g->clk.bit = PIN_CLK;
    g->dio.bit = PIN_DIO;
    tgt.state = STATE_UNSYNCED;

    memcpy(g->desc, "emulated SiM3U167", sizeof("emulated SiM3U167"));
    return 0;
}

void gpio_close(gpio_t *g)
{
    (void)g;
}

const char *gpio_desc(const gpio_t *g)
{
    return g->desc;
}
