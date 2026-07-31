/*
 * SPDX-FileCopyrightText: GARDENA GmbH
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/* MT7688 palmbus GPIO — the GARDENA smart gateway host */

#include "gpio_soc.h"

#include <stdint.h>

#define MT76X8_PALMBUS_PHYS 0x10000000ul
#define MT76X8_PALMBUS_SIZE 0x00100000u
#define MT76X8_GPIO_OFFSET 0x00000600u

/* Per-bank registers, 4 bytes apart: CTRL 1 = output */
#define MT_CTRL 0x00u
#define MT_DATA 0x20u
#define MT_DSET 0x30u
#define MT_DCLR 0x40u

static void mt7688_pin(gpio_pin_t *p, volatile uint32_t *gp, int bank, int bit)
{
    p->set = &gp[(MT_DSET >> 2) + bank];
    p->clr = &gp[(MT_DCLR >> 2) + bank];
    p->in = &gp[(MT_DATA >> 2) + bank];
    p->oe = &gp[(MT_CTRL >> 2) + bank];
    p->od = p->oe;
    p->dir_rmw = 1;
    p->bit = bit;
    p->mask = 1u << bit;
    p->port = gp;
}

static int mt7688_open(gpio_t *g, int cb, int cbit, int db, int dbit)
{
    volatile uint32_t *base =
        gpio_map_phys(g, MT76X8_PALMBUS_PHYS, MT76X8_PALMBUS_SIZE);
    if (!base) {
        return -1;
    }

    volatile uint32_t *gp =
        (volatile uint32_t *)((uint8_t *)base + MT76X8_GPIO_OFFSET);

    mt7688_pin(&g->clk, gp, cb, cbit);
    mt7688_pin(&g->dio, gp, db, dbit);

    /* Both lines start as inputs; the SWD layer drives them on connect */
    gpio_dir(&g->clk, 0);
    gpio_dir(&g->dio, 0);
    return 0;
}

static void mt7688_close(gpio_t *g)
{
    gpio_dir(&g->clk, 0);
    gpio_dir(&g->dio, 0);
}

static const char *const mt7688_ids[] = {"mt7688", "mt7628", "ralink", NULL};

const struct gpio_soc gpio_soc_mt7688 = {
    "mt7688", mt7688_ids, 3, 0, "36", "29", mt7688_open, mt7688_close,
};
