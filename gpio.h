/*
 * SPDX-FileCopyrightText: GARDENA GmbH
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef GPIO_H
#define GPIO_H

#include <stddef.h>
#include <stdint.h>

/* Bit-banged GPIO over direct MMIO.
 *
 * Each pin is resolved once, at open time, into the register pointers the
 * bit-bang loop needs.  Every edge is then a single store plus one read-back
 * for ordering — no per-bit dispatch, lookup or branch, so a second host SoC
 * costs nothing in the hot path. */

typedef struct {
    volatile uint32_t *set; /* write mask -> pin high */
    volatile uint32_t *clr; /* write mask -> pin low */
    volatile uint32_t *in; /* read -> pin levels; doubles as write barrier */
    volatile uint32_t *oe; /* direction, output (see dir_rmw) */
    volatile uint32_t *od; /* direction, input  (see dir_rmw) */
    uint32_t mask;
    int bit;
    int dir_rmw; /* 1: oe == od, a r/w register where 1 = output
                    0: oe/od are write-1-to-act registers */

    /* Backend bookkeeping, not used in the hot path */
    volatile uint32_t *port; /* register block this pin lives in */
    int was_muxed; /* pin was owned by a peripheral at open time */
} gpio_pin_t;

typedef struct {
    gpio_pin_t clk;
    gpio_pin_t dio;

    /* private */
    const struct gpio_soc *soc;
    void *map;
    size_t maplen;
    char desc[64];
} gpio_t;

/*
 * Hot path
 */

#ifdef GPIO_MOCK

/* Out of line, so a test can put a SWD target on the other end of the wire
   instead of /dev/mem.  Only test builds define GPIO_MOCK; see gpio_mock.c. */
void gpio_set(const gpio_pin_t *p);
void gpio_clr(const gpio_pin_t *p);
void gpio_put(const gpio_pin_t *p, int high);
void gpio_sync(const gpio_pin_t *p);
int gpio_get(const gpio_pin_t *p);
void gpio_dir(const gpio_pin_t *p, int output);

#else

static inline void gpio_set(const gpio_pin_t *p)
{
    *p->set = p->mask;
}
static inline void gpio_clr(const gpio_pin_t *p)
{
    *p->clr = p->mask;
}

static inline void gpio_put(const gpio_pin_t *p, int high)
{
    *(high ? p->set : p->clr) = p->mask;
}

/* Read-back of the pin register: flushes the posted write and provides the
   setup/hold delay between edges */
static inline void gpio_sync(const gpio_pin_t *p)
{
    (void)*p->in;
}

static inline int gpio_get(const gpio_pin_t *p)
{
    return (int)((*p->in >> p->bit) & 1u);
}

static inline void gpio_dir(const gpio_pin_t *p, int output)
{
    if (p->dir_rmw) {
        if (output) {
            *p->oe |= p->mask;
        } else {
            *p->oe &= ~p->mask;
        }
    } else {
        *(output ? p->oe : p->od) = p->mask;
    }
    gpio_sync(p);
}

#endif /* GPIO_MOCK */

/*
 * Setup / teardown
 */

/* Map the host SoC's GPIO block and claim the SWCLK/SWDIO pins.  The SoC is
   auto-detected; SWD_SOC, SWD_SWCLK and SWD_SWDIO override.  Both pins are
   left as inputs.  Returns 0 on success. */
int gpio_open(gpio_t *g);

/* Release the pins (back to input, and back to their peripheral if they were
   muxed to one) and unmap */
void gpio_close(gpio_t *g);

/* "at91sam9x5 SWCLK=PC12 SWDIO=PC11" — for diagnostics */
const char *gpio_desc(const gpio_t *g);

#endif /* GPIO_H */
