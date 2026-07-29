#include "swd.h"

#include "gpio.h"

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Extra dummy register reads per clock half-period (tunable via SWD_DELAY) */
static int g_swd_delay = 0;

/* AP 0 is the only AP this tool touches, so DP_SELECT only ever changes the
   register bank; cache it to skip redundant SELECT writes. */
#define AP_BANK_NONE 0xFFu

struct swd_ctx {
    gpio_t   io;
    uint8_t  ap_bank;        /* bank currently in DP_SELECT, or AP_BANK_NONE */
};

/* ------------------------------------------------------------------ */
/* Low-level GPIO bit-bang                                             */
/* ------------------------------------------------------------------ */

static inline void delay_reads(swd_ctx_t *g)
{
    for (int i = 0; i < g_swd_delay; i++)
        gpio_sync(&g->io.dio);
}

static inline void swdio_drive(swd_ctx_t *g, int output)
{
    gpio_dir(&g->io.dio, output);
}

/* Setup phase: flush the SWDIO write and hold the level before the edge */
static inline void swdio_settle(swd_ctx_t *g)
{
    gpio_sync(&g->io.dio);
    delay_reads(g);
}

/* Rising edge, high phase, falling edge — the target samples on the rise, so
   SWDIO must already be driven (or read) by the caller */
static inline void clock_pulse(swd_ctx_t *g)
{
    gpio_set(&g->io.clk);
    gpio_sync(&g->io.clk);
    delay_reads(g);
    gpio_clr(&g->io.clk);
}

/* Clock out one bit (MOSI) */
static inline void clock_out(swd_ctx_t *g, int bit)
{
    gpio_put(&g->io.dio, bit);
    swdio_settle(g);
    clock_pulse(g);
}

/* Clock in one bit (MISO) — sample before the rising edge */
static inline int clock_in(swd_ctx_t *g)
{
    swdio_settle(g);
    int val = gpio_get(&g->io.dio);
    clock_pulse(g);
    return val;
}

/* Send N idle clocks (SWDIO=0) */
static void clock_idle(swd_ctx_t *g, int n)
{
    gpio_clr(&g->io.dio);
    for (int i = 0; i < n; i++) {
        swdio_settle(g);
        clock_pulse(g);
    }
}

/* Send N bits LSB-first from a byte array */
static void send_bits(swd_ctx_t *g, const uint8_t *data, int n)
{
    for (int i = 0; i < n; i++)
        clock_out(g, (data[i / 8] >> (i % 8)) & 1);
}

/* Send 32-bit word LSB-first */
static void send_word(swd_ctx_t *g, uint32_t val)
{
    for (int i = 0; i < 32; i++) {
        clock_out(g, val & 1);
        val >>= 1;
    }
}

/* Receive 32-bit word LSB-first */
static uint32_t recv_word(swd_ctx_t *g)
{
    uint32_t val = 0;
    for (int i = 0; i < 32; i++)
        val |= (uint32_t)clock_in(g) << i;
    return val;
}

static int parity32(uint32_t v)
{
    v ^= v >> 16; v ^= v >> 8; v ^= v >> 4;
    v ^= v >> 2;  v ^= v >> 1;
    return v & 1;
}

/* ------------------------------------------------------------------ */
/* SWD request byte                                                    */
/* ------------------------------------------------------------------ */
/* APnDP: 0=DP 1=AP, RnW: 0=write 1=read, addr: register addr A[3:2] */
static uint8_t swd_request(int APnDP, int RnW, uint8_t addr)
{
    uint8_t req = 0x81u;  /* START=1, STOP=0, PARK=1 */
    if (APnDP) req |= (1u << 1);
    if (RnW)   req |= (1u << 2);
    req |= ((addr & 0x04u) ? (1u << 3) : 0);
    req |= ((addr & 0x08u) ? (1u << 4) : 0);
    /* parity over APnDP, RnW, A[3:2] */
    int p = APnDP ^ RnW ^ ((addr >> 2) & 1) ^ ((addr >> 3) & 1);
    if (p) req |= (1u << 5);
    return req;
}

/* ------------------------------------------------------------------ */
/* SWD transaction                                                     */
/* ------------------------------------------------------------------ */

/* Returns SWD_ACK_OK / SWD_ACK_WAIT / SWD_ACK_FAULT, or -1 on parity error */
static int swd_transfer(swd_ctx_t *ctx, uint8_t req, int write, uint32_t *data)
{
    /* Send request byte */
    swdio_drive(ctx, 1);
    for (int i = 0; i < 8; i++) {
        clock_out(ctx, (req >> i) & 1);
    }

    /* Turnaround: host releases SWDIO */
    swdio_drive(ctx, 0);
    clock_in(ctx);  /* 1 turnaround clock */

    /* Read 3-bit ACK (LSB first) */
    int ack = 0;
    for (int i = 0; i < 3; i++)
        ack |= clock_in(ctx) << i;

    if (ack == SWD_ACK_OK) {
        if (!write) {
            /* Read: receive data + parity while SWDIO still input */
            uint32_t val = recv_word(ctx);
            int par = clock_in(ctx);
            /* Turnaround back to MOSI */
            swdio_drive(ctx, 1);
            clock_out(ctx, 0);  /* 1 turnaround clock, SWDIO=0 */
            if (par != parity32(val)) {
                fprintf(stderr, "[!] SWD parity error on read\n");
                return -1;
            }
            *data = val;
        } else {
            /* Write: turnaround back to MOSI, then send data + parity */
            swdio_drive(ctx, 1);
            clock_out(ctx, 0);  /* 1 turnaround clock, SWDIO=0 */
            send_word(ctx, *data);
            clock_out(ctx, parity32(*data));
        }
    } else {
        /* WAIT or FAULT: overrun detection is disabled, so there is no
           data phase — just turn the bus around back to MOSI */
        swdio_drive(ctx, 1);
        clock_out(ctx, 0);  /* 1 turnaround clock, SWDIO=0 */
    }

    /* Idle clocks after transfer */
    clock_idle(ctx, SWD_IDLE_CLOCKS);
    return ack;
}

/* Clear sticky error flags via ABORT register */
static int swd_clear_errors(swd_ctx_t *ctx)
{
    uint8_t req = swd_request(0, 0, DP_ABORT);
    uint32_t val = DP_ABORT_ALL;
    return swd_transfer(ctx, req, 1, &val);
}

/* Run a transfer, retrying on WAIT (plain retry: WAIT is backpressure and
   sets no sticky flags). On FAULT, clear the sticky error flags via ABORT
   so the DP is usable again, then fail. */
static int swd_transfer_retry(swd_ctx_t *ctx, uint8_t req, int write,
                              uint32_t *data, const char *what)
{
    for (int retry = 0; retry < SWD_WAIT_RETRIES; retry++) {
        int ack = swd_transfer(ctx, req, write, data);
        if (ack == SWD_ACK_OK)   return 0;
        if (ack == SWD_ACK_WAIT) continue;
        if (ack == SWD_ACK_FAULT) swd_clear_errors(ctx);
        fprintf(stderr, "[!] %s (req=0x%02x) ack=%d\n", what, req, ack);
        return -1;
    }
    fprintf(stderr, "[!] %s (req=0x%02x): too many WAITs\n", what, req);
    return -1;
}

/* ------------------------------------------------------------------ */
/* DP register access                                                  */
/* ------------------------------------------------------------------ */

int swd_dp_read(swd_ctx_t *ctx, uint8_t addr, uint32_t *data)
{
    uint8_t req = swd_request(0, 1, addr);
    return swd_transfer_retry(ctx, req, 0, data, "DP read");
}

int swd_dp_write(swd_ctx_t *ctx, uint8_t addr, uint32_t data)
{
    uint8_t req = swd_request(0, 0, addr);
    return swd_transfer_retry(ctx, req, 1, &data, "DP write");
}

/* ------------------------------------------------------------------ */
/* AP register access                                                  */
/* ------------------------------------------------------------------ */

/* Point DP_SELECT at the bank holding addr[7:4] of AP 0 */
static int ap_select(swd_ctx_t *ctx, uint8_t addr)
{
    uint8_t bank = addr & 0xF0u;
    if (ctx->ap_bank == bank)
        return 0;
    if (swd_dp_write(ctx, DP_SELECT, bank) != 0)
        return -1;
    ctx->ap_bank = bank;
    return 0;
}

static int ap_read(swd_ctx_t *ctx, uint8_t addr, uint32_t *data)
{
    if (ap_select(ctx, addr) != 0) return -1;
    uint8_t req = swd_request(1, 1, addr & 0x0Cu);
    if (swd_transfer_retry(ctx, req, 0, data, "AP read") != 0) return -1;
    /* AP read returns posted result; read RDBUFF for actual value */
    return swd_dp_read(ctx, DP_RDBUFF, data);
}

static int ap_write(swd_ctx_t *ctx, uint8_t addr, uint32_t data)
{
    if (ap_select(ctx, addr) != 0) return -1;
    uint8_t req = swd_request(1, 0, addr & 0x0Cu);
    return swd_transfer_retry(ctx, req, 1, &data, "AP write");
}

/* ------------------------------------------------------------------ */
/* MEM-AP memory access                                                */
/* ------------------------------------------------------------------ */

static int memap_setup(swd_ctx_t *ctx)
{
    int r = ap_write(ctx, AP_CSW, AP_CSW_VAL);
    return r;
}

int swd_mem_write32(swd_ctx_t *ctx, uint32_t addr, uint32_t data)
{
    if (ap_write(ctx, AP_TAR, addr) != 0) return -1;
    return ap_write(ctx, AP_DRW, data);
}

int swd_mem_read32(swd_ctx_t *ctx, uint32_t addr, uint32_t *data)
{
    if (ap_write(ctx, AP_TAR, addr) != 0) return -1;
    return ap_read(ctx, AP_DRW, data);
}

/* Sequential block read using TAR auto-increment and pipelined DRW reads:
   one TAR write per 1 KB window (auto-increment is only architecturally
   guaranteed within 10 TAR bits), then back-to-back posted DRW reads where
   read N returns word N-1, and a final RDBUFF read for the last word. */
int swd_mem_read_block(swd_ctx_t *ctx, uint32_t addr, uint32_t *buf,
                       uint32_t count)
{
    uint32_t done = 0;
    while (done < count) {
        uint32_t chunk = (0x400u - (addr & 0x3FFu)) / 4;
        if (chunk > count - done) chunk = count - done;

        if (ap_write(ctx, AP_TAR, addr) != 0) return -1;
        if (ap_select(ctx, AP_DRW) != 0) return -1;

        uint8_t req = swd_request(1, 1, AP_DRW & 0x0Cu);
        uint32_t val;
        /* First DRW read is posted; its value arrives with the next read */
        if (swd_transfer_retry(ctx, req, 0, &val, "AP block read") != 0)
            return -1;
        for (uint32_t i = 1; i < chunk; i++) {
            if (swd_transfer_retry(ctx, req, 0, &val, "AP block read") != 0)
                return -1;
            buf[done + i - 1] = val;
        }
        if (swd_dp_read(ctx, DP_RDBUFF, &val) != 0) return -1;
        buf[done + chunk - 1] = val;

        done += chunk;
        addr += chunk * 4;
    }
    return 0;
}

/* Stream values into a single address: CSW auto-increment off so every DRW
   write lands on the same register, one TAR write, then back-to-back writes
   with no address phase in between.  A WAIT ack is the target's backpressure
   and swd_transfer_retry rides it out, so a slow slave throttles the stream
   instead of losing data.  RDBUFF at the end waits for the last posted write
   to retire before the caller assumes it happened. */
int swd_mem_write_fixed(swd_ctx_t *ctx, uint32_t addr, const uint32_t *vals,
                        uint32_t count)
{
    int ret = -1;

    if (ap_write(ctx, AP_CSW, AP_CSW_VAL_NOINC) != 0) goto out;
    if (ap_write(ctx, AP_TAR, addr) != 0) goto out;
    if (ap_select(ctx, AP_DRW) != 0) goto out;

    uint8_t req = swd_request(1, 0, AP_DRW & 0x0Cu);
    for (uint32_t i = 0; i < count; i++) {
        uint32_t val = vals[i];
        if (swd_transfer_retry(ctx, req, 1, &val, "AP fixed write") != 0)
            goto out;
    }

    uint32_t flush;
    if (swd_dp_read(ctx, DP_RDBUFF, &flush) != 0) goto out;

    ret = 0;

out:
    /* Leave the AP in the auto-increment mode every other caller expects */
    if (ap_write(ctx, AP_CSW, AP_CSW_VAL) != 0) ret = -1;
    return ret;
}

/* ------------------------------------------------------------------ */
/* Halt / run                                                          */
/* ------------------------------------------------------------------ */

#define DHCSR       0xE000EDF0u
#define DHCSR_DBGKEY    0xA05F0000u
#define DHCSR_C_DEBUGEN (1u << 0)
#define DHCSR_C_HALT    (1u << 1)
#define DHCSR_S_HALT    (1u << 17)

int swd_halt(swd_ctx_t *ctx)
{
    uint32_t val = DHCSR_DBGKEY | DHCSR_C_DEBUGEN | DHCSR_C_HALT;
    if (swd_mem_write32(ctx, DHCSR, val) != 0) return -1;
    for (int i = 0; i < 100; i++) {
        uint32_t stat;
        if (swd_mem_read32(ctx, DHCSR, &stat) != 0) return -1;
        if (stat & DHCSR_S_HALT) return 0;
    }
    fprintf(stderr, "[!] CPU halt timeout\n");
    return -1;
}

int swd_run(swd_ctx_t *ctx)
{
    uint32_t val = DHCSR_DBGKEY | DHCSR_C_DEBUGEN;
    return swd_mem_write32(ctx, DHCSR, val);
}

/* ------------------------------------------------------------------ */
/* Connect sequence                                                    */
/* ------------------------------------------------------------------ */

/* 118-bit JTAG-to-SWD sequence (from OpenOCD swd.h) */
static const uint8_t jtag_to_swd[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7b, 0x9e,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f,
};
/* 51-bit line reset */
static const uint8_t line_reset[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x03,
};

int swd_connect(swd_ctx_t *ctx, uint32_t *idcode)
{
    /* Ensure SWCLK starts low, SWDIO high — levels first, then drive, so
       neither line glitches as its output stage is enabled */
    gpio_clr(&ctx->io.clk);
    gpio_set(&ctx->io.dio);
    gpio_dir(&ctx->io.clk, 1);
    gpio_dir(&ctx->io.dio, 1);

    /* JTAG-to-SWD switch sequence */
    send_bits(ctx, jtag_to_swd, 118);
    /* Line reset */
    send_bits(ctx, line_reset, 51);
    /* 2 idle clocks */
    clock_idle(ctx, 2);

    /* Read IDCODE (first DP read after connect) */
    uint8_t req = swd_request(0, 1, DP_IDCODE);
    uint32_t id = 0;
    if (swd_transfer_retry(ctx, req, 0, &id, "IDCODE read") != 0)
        return -1;
    *idcode = id;

    /* Power up debug + system domains */
    if (swd_dp_write(ctx, DP_CTRL,
            DP_CTRL_CDBGPWRUPREQ | DP_CTRL_CSYSPWRUPREQ) != 0)
        return -1;
    uint32_t ctrl = 0;
    int powered = 0;
    for (int i = 0; i < 100; i++) {
        if (swd_dp_read(ctx, DP_CTRL, &ctrl) != 0) return -1;
        if ((ctrl & (DP_CTRL_CDBGPWRUPACK | DP_CTRL_CSYSPWRUPACK)) ==
                    (DP_CTRL_CDBGPWRUPACK | DP_CTRL_CSYSPWRUPACK)) {
            powered = 1;
            break;
        }
    }
    if (!powered) {
        fprintf(stderr, "[!] Debug power-up timeout (CTRL/STAT=0x%08x)\n",
                ctrl);
        return -1;
    }

    /* The line reset above zeroed DP_SELECT in the DP, so drop the cache */
    ctx->ap_bank = AP_BANK_NONE;
    return memap_setup(ctx);
}

/* ------------------------------------------------------------------ */
/* Open / close                                                        */
/* ------------------------------------------------------------------ */

swd_ctx_t *swd_open(void)
{
    const char *delay_env = getenv("SWD_DELAY");
    if (delay_env) g_swd_delay = atoi(delay_env);

    swd_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) return NULL;

    if (gpio_open(&ctx->io) != 0) {
        free(ctx);
        return NULL;
    }

    ctx->ap_bank = AP_BANK_NONE;
    return ctx;
}

void swd_close(swd_ctx_t *ctx)
{
    if (!ctx) return;
    gpio_close(&ctx->io);
    free(ctx);
}

const char *swd_host(const swd_ctx_t *ctx)
{
    return gpio_desc(&ctx->io);
}
