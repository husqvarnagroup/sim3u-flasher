/*
 * SPDX-FileCopyrightText: GARDENA GmbH
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/* Direct-MMIO bit-banged GPIO.
 *
 * This file holds the parts common to every host: the physical mapping, SoC
 * detection and pin-name parsing.  The register-level backends live in
 * gpio_<soc>.c and are reached through the gpio_soc table below.
 *
 * The SoC is auto-detected from the device tree; SWD_SOC overrides.  Pins
 * default to the wiring of each board and are overridable via SWD_SWCLK /
 * SWD_SWDIO, which accept "PC12", "pioC12" or a Linux-style pin number. */

#define _FILE_OFFSET_BITS 64 /* AT91 register block sits above 2 GB */

#include "gpio.h"
#include "gpio_soc.h"

#include <ctype.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/mman.h>
#include <unistd.h>

/*
 * mmap helper
 */

volatile uint32_t *gpio_map_phys(gpio_t *g, unsigned long phys, size_t len)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("open /dev/mem");
        return NULL;
    }
    void *base =
        mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, (off_t)phys);
    close(fd);
    if (base == MAP_FAILED) {
        perror("mmap");
        return NULL;
    }
    g->map = base;
    g->maplen = len;
    return (volatile uint32_t *)base;
}

/*
 * SoC table
 */

static const struct gpio_soc *const socs[] = {
    &gpio_soc_mt7688,
    &gpio_soc_at91sam9x5,
};

#define NSOCS ((int)(sizeof(socs) / sizeof(socs[0])))

/*
 * Detection
 */

/* /proc/device-tree/compatible is a list of NUL-separated strings; cpuinfo is
   plain text.  Both are read raw and lowercased for substring matching. */
static int file_has(const char *path, const char *const *needles)
{
    char buf[4096];
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        return 0;
    }
    ssize_t n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (n <= 0) {
        return 0;
    }

    for (ssize_t i = 0; i < n; i++) {
        if (buf[i] == '\0') {
            buf[i] = '\n';
        } else {
            buf[i] = (char)tolower((unsigned char)buf[i]);
        }
    }
    buf[n] = '\0';

    for (int i = 0; needles[i]; i++) {
        if (strstr(buf, needles[i])) {
            return 1;
        }
    }
    return 0;
}

static const struct gpio_soc *detect_soc(void)
{
    const char *env = getenv("SWD_SOC");
    if (env) {
        for (int i = 0; i < NSOCS; i++) {
            if (strcasecmp(env, socs[i]->name) == 0) {
                return socs[i];
            }
        }
        fprintf(stderr, "[!] Unknown SWD_SOC '%s'\n", env);
        return NULL;
    }

    for (int i = 0; i < NSOCS; i++) {
        if (file_has("/proc/device-tree/compatible", socs[i]->ids) ||
            file_has("/proc/cpuinfo", socs[i]->ids)) {
            return socs[i];
        }
    }

    fprintf(stderr, "[!] Could not detect host SoC — set SWD_SOC to one of:");
    for (int i = 0; i < NSOCS; i++) {
        fprintf(stderr, " %s", socs[i]->name);
    }
    fprintf(stderr, "\n");
    return NULL;
}

/*
 * Pin names
 */

/* "PC12", "pioC12", "gpio36", "36" -> bank + bit */
static int parse_pin(const char *s, int *bank, int *bit)
{
    static const char *const prefixes[] = {"gpio", "pio", "gp", "p", NULL};
    const char *p = s;

    while (isspace((unsigned char)*p)) {
        p++;
    }

    for (int i = 0; prefixes[i]; i++) {
        size_t n = strlen(prefixes[i]);
        if (strncasecmp(p, prefixes[i], n) == 0 && p[n] != '\0') {
            p += n;
            break;
        }
    }

    if (isalpha((unsigned char)*p)) {
        int b = toupper((unsigned char)*p) - 'A';
        p++;
        if (!isdigit((unsigned char)*p)) {
            return -1;
        }
        long v = strtol(p, NULL, 10);
        if (b < 0 || v < 0 || v > 31) {
            return -1;
        }
        *bank = b;
        *bit = (int)v;
        return 0;
    }

    if (!isdigit((unsigned char)*p)) {
        return -1;
    }
    long v = strtol(p, NULL, 10);
    if (v < 0 || v > 1023) {
        return -1;
    }
    *bank = (int)(v / 32);
    *bit = (int)(v % 32);
    return 0;
}

static int pin_from_env(const struct gpio_soc *soc, const char *env,
                        const char *def, int *bank, int *bit)
{
    const char *val = getenv(env);
    if (!val) {
        val = def;
    }

    if (parse_pin(val, bank, bit) != 0) {
        fprintf(stderr, "[!] Bad pin '%s' in %s\n", val, env);
        return -1;
    }
    if (*bank >= soc->nbanks) {
        fprintf(stderr, "[!] Pin '%s' out of range for %s (%d bank(s))\n", val,
                soc->name, soc->nbanks);
        return -1;
    }
    return 0;
}

static void pin_name(const struct gpio_soc *soc, int bank, int bit, char *out,
                     size_t len)
{
    if (soc->lettered) {
        snprintf(out, len, "P%c%d", 'A' + bank, bit);
    } else {
        snprintf(out, len, "%d", bank * 32 + bit);
    }
}

/*
 * Open / close
 */

int gpio_open(gpio_t *g)
{
    memset(g, 0, sizeof(*g));

    const struct gpio_soc *soc = detect_soc();
    if (!soc) {
        return -1;
    }
    g->soc = soc;

    int cb, cbit, db, dbit;
    if (pin_from_env(soc, "SWD_SWCLK", soc->def_clk, &cb, &cbit) != 0) {
        return -1;
    }
    if (pin_from_env(soc, "SWD_SWDIO", soc->def_dio, &db, &dbit) != 0) {
        return -1;
    }

    if (cb == db && cbit == dbit) {
        fprintf(stderr, "[!] SWCLK and SWDIO are the same pin\n");
        return -1;
    }

    if (soc->open(g, cb, cbit, db, dbit) != 0) {
        return -1;
    }

    char cname[16], dname[16];
    pin_name(soc, cb, cbit, cname, sizeof(cname));
    pin_name(soc, db, dbit, dname, sizeof(dname));
    snprintf(g->desc, sizeof(g->desc), "%s SWCLK=%s SWDIO=%s", soc->name, cname,
             dname);
    return 0;
}

void gpio_close(gpio_t *g)
{
    if (!g->soc) {
        return;
    }
    g->soc->close(g);
    if (g->map) {
        munmap(g->map, g->maplen);
    }
    g->map = NULL;
    g->soc = NULL;
}

const char *gpio_desc(const gpio_t *g)
{
    return g->desc;
}
