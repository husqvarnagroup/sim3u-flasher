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
 * Three environment variables bend the target's behaviour so the error paths
 * can be reached, since nothing else provokes them:
 *
 *   SWD_MOCK_IDCODE      report this IDCODE instead of the SiM3U167's
 *   SWD_MOCK_WAIT_EVERY  answer every Nth transfer with WAIT
 *   SWD_MOCK_FAULT_AT    answer the Nth transfer with FAULT
 *
 * A WAIT is backpressure the host is expected to ride out, so a run with
 * WAIT injection still has to finish; a FAULT is not, so it must not. */

#include "gpio.h"
#include "swd.h"

#include <stdint.h>
#include <stdlib.h>
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

/* The device's own register map, spelled out rather than taken from
   sim3u_flash.h.  An emulator that imports the addresses it is being driven
   with cannot notice one of them being wrong. */
#define FLASH_SIZE (256u * 1024u)
#define FLASH_PAGE 1024u

#define REG_FLASH_CONFIG_ALL 0x4002E000u
#define REG_FLASH_CONFIG_SET 0x4002E004u
#define REG_FLASH_CONFIG_CLR 0x4002E008u
#define REG_FLASH_WRADDR 0x4002E0A0u
#define REG_FLASH_WRDATA 0x4002E0B0u
#define REG_FLASH_KEY 0x4002E0C0u
#define REG_DHCSR 0xE000EDF0u

#define CONFIG_ERASEEN 0x00040000u
#define CONFIG_BUSYF 0x00100000u

#define KEY_INITIAL 0xA5u
#define KEY_SINGLE 0xF1u
#define KEY_MULTIPLE 0xF2u
#define KEY_LOCK 0x5Au

#define DHCSR_C_HALT (1u << 1)
#define DHCSR_S_HALT (1u << 17)

/* MEM-AP register addresses within bank 0 */
#define MEMAP_CSW 0x00u
#define MEMAP_TAR 0x04u
#define MEMAP_DRW 0x0Cu

/* CSW[5:4], the address auto-increment field */
#define CSW_ADDRINC(csw) (((csw) >> 4) & 3u)

enum key_state {
    KEY_LOCKED,
    KEY_STARTED, /* KEY_INITIAL seen */
    KEY_ONE_SHOT, /* armed for a single erase or write */
    KEY_STREAM, /* armed until locked again */
};

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

    /* MEM-AP */
    uint32_t csw;
    uint32_t tar;
    uint32_t rdbuff; /* the posted read waiting to be collected */

    /* Flash controller */
    uint32_t config;
    uint32_t wraddr;
    enum key_state key;

    uint32_t dhcsr;

    /* Fault injection */
    uint32_t idcode;
    unsigned long transfers;
    unsigned long wait_every;
    unsigned long fault_at;
} tgt;

static uint8_t flash[FLASH_SIZE];

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
 * Flash controller
 */

/* A key sequence arms the controller for one operation or for a stream of
   them.  Anything unexpected, KEY_LOCK included, locks it again. */
static void flash_key(uint32_t val)
{
    if (val == KEY_INITIAL) {
        tgt.key = KEY_STARTED;
    } else if (val == KEY_SINGLE && tgt.key == KEY_STARTED) {
        tgt.key = KEY_ONE_SHOT;
    } else if (val == KEY_MULTIPLE && tgt.key == KEY_STARTED) {
        tgt.key = KEY_STREAM;
    } else {
        tgt.key = KEY_LOCKED;
    }
}

/* Programming can only clear bits, which is what erasing exists to undo.  A
   run that writes without erasing first therefore fails to verify, exactly as
   the part would. */
static void flash_wrdata(uint32_t val)
{
    if (tgt.key != KEY_ONE_SHOT && tgt.key != KEY_STREAM) {
        return;
    }

    if (tgt.config & CONFIG_ERASEEN) {
        uint32_t page = tgt.wraddr & ~(FLASH_PAGE - 1u);
        if (page + FLASH_PAGE <= FLASH_SIZE) {
            memset(&flash[page], 0xFF, FLASH_PAGE);
        }
    } else if (tgt.wraddr + 1 < FLASH_SIZE) {
        flash[tgt.wraddr] &= (uint8_t)(val & 0xFFu);
        flash[tgt.wraddr + 1] &= (uint8_t)((val >> 8) & 0xFFu);
        /* WRADDR walks forward on its own in multiple-write mode */
        tgt.wraddr += 2;
    }

    if (tgt.key == KEY_ONE_SHOT) {
        tgt.key = KEY_LOCKED;
    }
}

/*
 * Memory
 */

static uint32_t mem_read(uint32_t addr)
{
    if (addr + 3 < FLASH_SIZE) {
        uint32_t val;
        memcpy(&val, &flash[addr], sizeof(val));
        return val;
    }
    switch (addr) {
    case REG_FLASH_CONFIG_ALL:
        /* Never busy: the emulated flash programs instantly */
        return tgt.config & ~CONFIG_BUSYF;
    case REG_FLASH_WRADDR:
        return tgt.wraddr;
    case REG_DHCSR:
        return tgt.dhcsr | ((tgt.dhcsr & DHCSR_C_HALT) ? DHCSR_S_HALT : 0);
    default:
        return 0;
    }
}

static void mem_write(uint32_t addr, uint32_t val)
{
    switch (addr) {
    case REG_FLASH_CONFIG_SET:
        tgt.config |= val;
        break;
    case REG_FLASH_CONFIG_CLR:
        tgt.config &= ~val;
        break;
    case REG_FLASH_WRADDR:
        tgt.wraddr = val;
        break;
    case REG_FLASH_KEY:
        flash_key(val);
        break;
    case REG_FLASH_WRDATA:
        flash_wrdata(val);
        break;
    case REG_DHCSR:
        tgt.dhcsr = val;
        break;
    default:
        /* Flash is not writable through the MEM-AP, and the watchdog and
           clock registers only have to accept what sim3u_init sends */
        break;
    }
}

/*
 * DP and AP registers
 */

/* Auto-increment is only architecturally guaranteed across the bottom 10
   address bits, and a real MEM-AP wraps inside that window rather than
   carrying into the bits above it.  Modelling the wrap is what makes a
   caller that fails to rewrite TAR every 1 KB read the wrong words. */
static void tar_advance(void)
{
    if (CSW_ADDRINC(tgt.csw)) {
        tgt.tar = (tgt.tar & ~0x3FFu) | ((tgt.tar + 4) & 0x3FFu);
    }
}

static uint32_t read_ap(uint8_t addr)
{
    switch (addr) {
    case MEMAP_CSW:
        return tgt.csw;
    case MEMAP_TAR:
        return tgt.tar;
    case MEMAP_DRW: {
        /* Reads are posted: this returns the previous one and starts a new
           one, which the host collects from RDBUFF or the next read */
        uint32_t posted = tgt.rdbuff;
        tgt.rdbuff = mem_read(tgt.tar);
        tar_advance();
        return posted;
    }
    default:
        return 0;
    }
}

static void write_ap(uint8_t addr, uint32_t val)
{
    switch (addr) {
    case MEMAP_CSW:
        tgt.csw = val;
        break;
    case MEMAP_TAR:
        tgt.tar = val;
        break;
    case MEMAP_DRW:
        mem_write(tgt.tar, val);
        tar_advance();
        break;
    default:
        break;
    }
}

static uint32_t read_reg(int apndp, uint8_t addr)
{
    if (apndp) {
        return read_ap(addr);
    }
    switch (addr) {
    case DP_IDCODE:
        return tgt.idcode;
    case DP_CTRL:
        return tgt.ctrl_stat;
    case DP_RDBUFF:
        return tgt.rdbuff;
    default:
        return 0;
    }
}

static void write_reg(int apndp, uint8_t addr, uint32_t val)
{
    if (apndp) {
        write_ap(addr, val);
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
   first */
static void queue_ack(int ack)
{
    tgt.out_len = 0;
    tgt.out_pos = 0;
    queue_bit(0);
    for (int i = 0; i < 3; i++) {
        queue_bit((ack >> i) & 1);
    }
}

/* WAIT and FAULT both end the transfer after the ack, with no data phase */
static int injected_ack(void)
{
    tgt.transfers++;
    if (tgt.fault_at && tgt.transfers == tgt.fault_at) {
        return SWD_ACK_FAULT;
    }
    if (tgt.wait_every && tgt.transfers % tgt.wait_every == 0) {
        return SWD_ACK_WAIT;
    }
    return SWD_ACK_OK;
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

    int ack = injected_ack();
    queue_ack(ack);
    if (ack != SWD_ACK_OK) {
        return;
    }

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

static unsigned long env_num(const char *name, unsigned long def)
{
    const char *val = getenv(name);
    return val ? strtoul(val, NULL, 0) : def;
}

int gpio_open(gpio_t *g)
{
    memset(g, 0, sizeof(*g));
    memset(&tgt, 0, sizeof(tgt));

    g->clk.bit = PIN_CLK;
    g->dio.bit = PIN_DIO;
    tgt.state = STATE_UNSYNCED;
    tgt.idcode = (uint32_t)env_num("SWD_MOCK_IDCODE", TARGET_IDCODE);
    tgt.wait_every = env_num("SWD_MOCK_WAIT_EVERY", 0);
    tgt.fault_at = env_num("SWD_MOCK_FAULT_AT", 0);

    /* Deliberately not 0xFF: flash comes up holding something, so a run that
       skips the erase cannot go on to verify */
    memset(flash, 0x5A, sizeof(flash));

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
