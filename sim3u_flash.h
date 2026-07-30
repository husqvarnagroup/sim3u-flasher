/*
 * SPDX-FileCopyrightText: GARDENA GmbH
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef SIM3U_FLASH_H
#define SIM3U_FLASH_H

#include <stdint.h>
#include "swd.h"

#define SIM3U_FLASH_CONFIG_ALL 0x4002E000u
#define SIM3U_FLASH_CONFIG_SET 0x4002E004u
#define SIM3U_FLASH_CONFIG_CLR 0x4002E008u
#define SIM3U_FLASH_WRADDR 0x4002E0A0u
#define SIM3U_FLASH_WRDATA 0x4002E0B0u
#define SIM3U_FLASH_KEY 0x4002E0C0u

#define SIM3U_FLASH_CONFIG_ERASEEN 0x00040000u
#define SIM3U_FLASH_CONFIG_BUSYF 0x00100000u

#define SIM3U_FLASH_KEY_INITIAL 0xA5u
#define SIM3U_FLASH_KEY_SINGLE 0xF1u
#define SIM3U_FLASH_KEY_MULTIPLE 0xF2u
#define SIM3U_FLASH_KEY_LOCK 0x5Au

#define SIM3U_FLASH_PAGE_SIZE 1024u
#define SIM3U_FLASH_BASE 0x00000000u
#define SIM3U_FLASH_SIZE_256K (256u * 1024u)
#define SIM3U_LOCK_WORD_ADDR 0x0003FFFCu
#define SIM3U_LOCK_WORD_UNLOCKED 0xFFFFFFFFu

#define SIM3U_DEVICEID0 0x400490C0u
#define SIM3U_EXPECTED_IDCODE 0x2BA01477u

#define SIM3U_WDTIMER0_CONTROL_SET 0x40030004u
#define SIM3U_WDTIMER0_WDTKEY 0x40030030u
#define SIM3U_WDTIMER0_KEY1 0xA5u
#define SIM3U_WDTIMER0_KEY2 0xDDu
#define SIM3U_WDTIMER0_DBGMD 0x00000010u

#define SIM3U_CLKCTRL0_APBCLKG0_SET 0x4002D024u
#define SIM3U_CLKCTRL0_FLCTRLCEN 0x40000000u

#define SIM3U_FLASH_BUSY_TIMEOUT 5000

/* Initialise device: disable watchdog, enable flash clock */
int sim3u_init(swd_ctx_t *ctx);

/* Erase all pages covered by [0, size) bytes */
int sim3u_erase(swd_ctx_t *ctx, uint32_t size);

/* Write data to flash starting at address 0 */
int sim3u_write(swd_ctx_t *ctx, const uint8_t *data, uint32_t size);

/* Verify flash matches data */
int sim3u_verify(swd_ctx_t *ctx, const uint8_t *data, uint32_t size);

#endif /* SIM3U_FLASH_H */
