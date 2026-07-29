#ifndef SWD_H
#define SWD_H

#include <stdint.h>

/* ACK values */
#define SWD_ACK_OK      1
#define SWD_ACK_WAIT    2
#define SWD_ACK_FAULT   4

/* DP register addresses (A[3:2]) */
#define DP_IDCODE   0x00
#define DP_ABORT    0x00  /* write */
#define DP_CTRL     0x04
#define DP_SELECT   0x08
#define DP_RDBUFF   0x0C

/* AP register addresses (A[3:2]) — MEM-AP */
#define AP_CSW      0x00
#define AP_TAR      0x04
#define AP_DRW      0x0C

/* DP CTRL/STAT bits */
#define DP_CTRL_CDBGPWRUPREQ  (1u << 28)
#define DP_CTRL_CSYSPWRUPREQ  (1u << 30)
#define DP_CTRL_CDBGPWRUPACK  (1u << 29)
#define DP_CTRL_CSYSPWRUPACK  (1u << 31)

/* MEM-AP CSW: 32-bit word transfers, auto-increment */
#define AP_CSW_VAL  0x23000052u

/* ABORT register bits */
#define DP_ABORT_STKCMPCLR  (1u << 1)
#define DP_ABORT_STKERRCLR  (1u << 2)
#define DP_ABORT_WDERRCLR   (1u << 3)
#define DP_ABORT_ORUNERRCLR (1u << 4)
#define DP_ABORT_ALL        (DP_ABORT_STKCMPCLR | DP_ABORT_STKERRCLR | \
                             DP_ABORT_WDERRCLR  | DP_ABORT_ORUNERRCLR)

/* SWD max WAIT retries */
#define SWD_WAIT_RETRIES  10

typedef struct swd_ctx swd_ctx_t;

/* Host SoC and pins are detected at open time; see gpio.c for the overrides */
swd_ctx_t *swd_open(void);
void       swd_close(swd_ctx_t *ctx);

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

/* Halt/resume CPU via DHCSR */
int swd_halt(swd_ctx_t *ctx);
int swd_run(swd_ctx_t *ctx);

#endif /* SWD_H */
