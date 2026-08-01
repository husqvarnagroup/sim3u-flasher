/*
 * SPDX-FileCopyrightText: GARDENA GmbH
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef SWD_H
#define SWD_H

#include <stdint.h>

/* ACK values */
#define SWD_ACK_OK 1
#define SWD_ACK_WAIT 2
#define SWD_ACK_FAULT 4

/* DP register addresses (A[3:2]) */
#define DP_IDCODE 0x00
#define DP_ABORT 0x00 /* write */
#define DP_CTRL 0x04
#define DP_SELECT 0x08
#define DP_RDBUFF 0x0C

/* AP register addresses (A[3:2]) — MEM-AP */
#define AP_CSW 0x00
#define AP_TAR 0x04
#define AP_DRW 0x0C

/* DP CTRL/STAT bits */
#define DP_CTRL_CDBGPWRUPREQ (1u << 28)
#define DP_CTRL_CSYSPWRUPREQ (1u << 30)
#define DP_CTRL_CDBGPWRUPACK (1u << 29)
#define DP_CTRL_CSYSPWRUPACK (1u << 31)

/* MEM-AP CSW: 32-bit word transfers, auto-increment */
#define AP_CSW_VAL 0x23000052u
/* Same, with address auto-increment off (repeated access to one register) */
#define AP_CSW_VAL_NOINC 0x23000042u

/* ABORT register bits */
#define DP_ABORT_STKCMPCLR (1u << 1)
#define DP_ABORT_STKERRCLR (1u << 2)
#define DP_ABORT_WDERRCLR (1u << 3)
#define DP_ABORT_ORUNERRCLR (1u << 4)
#define DP_ABORT_ALL                                               \
    (DP_ABORT_STKCMPCLR | DP_ABORT_STKERRCLR | DP_ABORT_WDERRCLR | \
     DP_ABORT_ORUNERRCLR)

/* Idle clocks appended to every transfer, letting a posted write retire before
   the next request starts.  They look like pure overhead (~15% of the bits on
   the wire) but are not: measured on GARDENA-01c737, dropping 8 to 2 left the
   mean flash time worse (10.0 s vs 8.8 s over three interleaved runs), because
   the target takes the time either way and charges it back as WAIT acks, and a
   retried transfer costs ~54 bit-times against an idle clock's one. */
#define SWD_IDLE_CLOCKS 8

/* Max WAIT acks to ride out before giving up on a transfer.  A WAIT is the
   target saying "busy", and each retry costs one SWD transfer (tens of us), so
   this is really a time budget: it has to cover the longest stall the target
   can produce, which is a flash halfword program on a module still running the
   slow post-reset clock.  Ten retries only ever worked against a module that
   was already executing firmware and had clocked itself up. */
#define SWD_WAIT_RETRIES 100000

typedef struct swd_ctx swd_ctx_t;

/* Host SoC and pins are detected at open time; see gpio.c for the overrides */
swd_ctx_t *swd_open(void);
void swd_close(swd_ctx_t *ctx);

/* "at91sam9x5 SWCLK=PC12 SWDIO=PC11" */
const char *swd_host(const swd_ctx_t *ctx);

/* SWD connect sequence: line reset + JTAG-to-SWD + read IDCODE */
int swd_connect(swd_ctx_t *ctx, uint32_t *idcode);

/* DP register access */
int swd_dp_read(swd_ctx_t *ctx, uint8_t addr, uint32_t *data);
int swd_dp_write(swd_ctx_t *ctx, uint8_t addr, uint32_t data);

/* MEM-AP memory access (sets up TAR, reads/writes DRW) */
int swd_mem_read32(swd_ctx_t *ctx, uint32_t addr, uint32_t *data);
int swd_mem_write32(swd_ctx_t *ctx, uint32_t addr, uint32_t data);

/* Sequential word block read (TAR auto-increment + pipelined DRW reads) */
int swd_mem_read_block(swd_ctx_t *ctx, uint32_t addr, uint32_t *buf,
                       uint32_t count);

/* Repeated writes to one address (TAR set once, auto-increment off) */
int swd_mem_write_fixed(swd_ctx_t *ctx, uint32_t addr, const uint32_t *vals,
                        uint32_t count);

/* Halt/resume CPU via DHCSR */
int swd_halt(swd_ctx_t *ctx);
int swd_run(swd_ctx_t *ctx);

#endif /* SWD_H */
