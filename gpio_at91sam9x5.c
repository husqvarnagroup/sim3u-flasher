/*
 * SPDX-FileCopyrightText: GARDENA GmbH
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/* AT91SAM9X5 PIO controller */

#include "gpio_soc.h"

#include <stdint.h>

/* One page covers PIOA..PIOD and the PMC */
#define AT91_SYSC_PHYS 0xFFFFF000ul
#define AT91_SYSC_SIZE 0x00001000u
#define AT91_PIOA_OFF 0x0400u
#define AT91_PIO_STRIDE 0x0200u
#define AT91_PMC_OFF 0x0C00u

#define PIO_PER 0x00u /* PIO enable: take pin from the peripheral mux */
#define PIO_PDR 0x04u /* PIO disable: hand it back */
#define PIO_PSR 0x08u /* 1 = pin is under PIO control */
#define PIO_OER 0x10u /* output enable */
#define PIO_ODR 0x14u /* output disable (input) */
#define PIO_IFDR 0x24u /* input glitch filter disable */
#define PIO_SODR 0x30u /* set output data */
#define PIO_CODR 0x34u /* clear output data */
#define PIO_PDSR 0x3Cu /* pin data status */
#define PIO_IDR 0x44u /* interrupt disable */
#define PIO_MDDR 0x54u /* multi-driver disable: push-pull, not open-drain */

#define PMC_PCER 0x10u
#define PMC_PCSR 0x18u

/* Peripheral IDs: PIOA/PIOB share 2, PIOC/PIOD share 3 */
#define AT91_PIO_PID(bank) ((bank) < 2 ? 2u : 3u)

static void at91_clk_enable(volatile uint32_t *pmc, int bank)
{
    uint32_t m = 1u << AT91_PIO_PID(bank);
    /* Reading PDSR needs the PIO clock running.  Linux normally leaves it on;
       enable it defensively rather than sample all-ones. */
    if (!(pmc[PMC_PCSR >> 2] & m)) {
        pmc[PMC_PCER >> 2] = m;
    }
}

static void at91_pin(gpio_pin_t *p, volatile uint32_t *port, int bit)
{
    uint32_t m = 1u << bit;

    /* Remember pins we take from a peripheral so close() can hand them back */
    p->was_muxed = !(port[PIO_PSR >> 2] & m);

    port[PIO_IDR >> 2] = m; /* no interrupts */
    port[PIO_IFDR >> 2] = m; /* no glitch filter — it would eat sampled bits */
    port[PIO_MDDR >> 2] = m; /* push-pull: open-drain cannot drive SWCLK high */
    port[PIO_PER >> 2] = m; /* claim the pin for PIO */
    port[PIO_ODR >> 2] = m; /* input until the SWD layer drives it */

    p->set = &port[PIO_SODR >> 2];
    p->clr = &port[PIO_CODR >> 2];
    p->in = &port[PIO_PDSR >> 2];
    p->oe = &port[PIO_OER >> 2];
    p->od = &port[PIO_ODR >> 2];
    p->dir_rmw = 0;
    p->bit = bit;
    p->mask = m;
    p->port = port;
}

static volatile uint32_t *at91_port(volatile uint32_t *base, int bank)
{
    return (volatile uint32_t *)((uint8_t *)base + AT91_PIOA_OFF +
                                 (unsigned)bank * AT91_PIO_STRIDE);
}

static int at91_open(gpio_t *g, int cb, int cbit, int db, int dbit)
{
    volatile uint32_t *base = gpio_map_phys(g, AT91_SYSC_PHYS, AT91_SYSC_SIZE);
    if (!base) {
        return -1;
    }

    volatile uint32_t *pmc =
        (volatile uint32_t *)((uint8_t *)base + AT91_PMC_OFF);

    at91_clk_enable(pmc, cb);
    at91_clk_enable(pmc, db);

    at91_pin(&g->clk, at91_port(base, cb), cbit);
    at91_pin(&g->dio, at91_port(base, db), dbit);
    return 0;
}

/* Back to input, and back to the peripheral for pins that were muxed to one */
static void at91_release(const gpio_pin_t *p)
{
    p->port[PIO_ODR >> 2] = p->mask;
    if (p->was_muxed) {
        p->port[PIO_PDR >> 2] = p->mask;
    }
}

static void at91_close(gpio_t *g)
{
    at91_release(&g->clk);
    at91_release(&g->dio);
}

static const char *const at91_ids[] = {"at91sam9", NULL};

const struct gpio_soc gpio_soc_at91sam9x5 = {
    "at91sam9x5", at91_ids, 4, 1, "PC12", "PC11", at91_open, at91_close,
};
