/*
 * SPDX-FileCopyrightText: GARDENA GmbH
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef GPIO_SOC_H
#define GPIO_SOC_H

#include <stddef.h>
#include <stdint.h>

#include "gpio.h"

/* Interface between the generic GPIO front end and the per-SoC backends.
 * Internal to the gpio_*.c files; users of the pins include gpio.h only. */

struct gpio_soc {
    const char *name;
    const char *const *ids; /* device-tree / cpuinfo substrings, lowercase */
    int nbanks;
    int lettered; /* pins are named PC12 rather than 76 */
    const char *def_clk;
    const char *def_dio;
    int (*open)(gpio_t *g, int cb, int cbit, int db, int dbit);
    void (*close)(gpio_t *g);
};

/* Map len bytes of physical address space and record the mapping in g so
   gpio_close() can unmap it.  Returns NULL on failure. */
volatile uint32_t *gpio_map_phys(gpio_t *g, unsigned long phys, size_t len);

extern const struct gpio_soc gpio_soc_mt7688;
extern const struct gpio_soc gpio_soc_at91sam9x5;

#endif /* GPIO_SOC_H */
