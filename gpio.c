/* Direct-MMIO bit-banged GPIO backends.
 *
 * Two host SoCs drive the SWD lines:
 *   mt7688      GARDENA smart gateway, palmbus GPIO block
 *   at91sam9x5  Atmel PIO controller
 *
 * The SoC is auto-detected from the device tree; SWD_SOC overrides.  Pins
 * default to the wiring of each board and are overridable via SWD_SWCLK /
 * SWD_SWDIO, which accept "PC12", "pioC12" or a Linux-style pin number. */

#define _FILE_OFFSET_BITS 64   /* AT91 register block sits above 2 GB */

#include "gpio.h"

#include <ctype.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/mman.h>
#include <unistd.h>

struct gpio_soc {
    const char *name;
    const char *const *ids;    /* device-tree / cpuinfo substrings, lowercase */
    int   nbanks;
    int   lettered;            /* pins are named PC12 rather than 76 */
    const char *def_clk;
    const char *def_dio;
    int  (*open)(gpio_t *g, int cb, int cbit, int db, int dbit);
    void (*close)(gpio_t *g);
};

/* ------------------------------------------------------------------ */
/* mmap helper                                                         */
/* ------------------------------------------------------------------ */

static volatile uint32_t *map_phys(gpio_t *g, unsigned long phys, size_t len)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("open /dev/mem");
        return NULL;
    }
    void *base = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                      (off_t)phys);
    close(fd);
    if (base == MAP_FAILED) {
        perror("mmap");
        return NULL;
    }
    g->map    = base;
    g->maplen = len;
    return (volatile uint32_t *)base;
}

/* ------------------------------------------------------------------ */
/* MT7688 palmbus GPIO                                                 */
/* ------------------------------------------------------------------ */

#define MT76X8_PALMBUS_PHYS 0x10000000ul
#define MT76X8_PALMBUS_SIZE 0x00100000u
#define MT76X8_GPIO_OFFSET  0x00000600u

/* Per-bank registers, 4 bytes apart: CTRL 1 = output */
#define MT_CTRL 0x00u
#define MT_DATA 0x20u
#define MT_DSET 0x30u
#define MT_DCLR 0x40u

static void mt7688_pin(gpio_pin_t *p, volatile uint32_t *gp, int bank, int bit)
{
    p->set = &gp[(MT_DSET >> 2) + bank];
    p->clr = &gp[(MT_DCLR >> 2) + bank];
    p->in  = &gp[(MT_DATA >> 2) + bank];
    p->oe  = &gp[(MT_CTRL >> 2) + bank];
    p->od  = p->oe;
    p->dir_rmw = 1;
    p->bit  = bit;
    p->mask = 1u << bit;
    p->port = gp;
}

static int mt7688_open(gpio_t *g, int cb, int cbit, int db, int dbit)
{
    volatile uint32_t *base = map_phys(g, MT76X8_PALMBUS_PHYS,
                                       MT76X8_PALMBUS_SIZE);
    if (!base) return -1;

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

/* ------------------------------------------------------------------ */
/* AT91SAM9X5 PIO                                                      */
/* ------------------------------------------------------------------ */

/* One page covers PIOA..PIOD and the PMC */
#define AT91_SYSC_PHYS  0xFFFFF000ul
#define AT91_SYSC_SIZE  0x00001000u
#define AT91_PIOA_OFF   0x0400u
#define AT91_PIO_STRIDE 0x0200u
#define AT91_PMC_OFF    0x0C00u

#define PIO_PER  0x00u   /* PIO enable: take pin from the peripheral mux */
#define PIO_PDR  0x04u   /* PIO disable: hand it back */
#define PIO_PSR  0x08u   /* 1 = pin is under PIO control */
#define PIO_OER  0x10u   /* output enable */
#define PIO_ODR  0x14u   /* output disable (input) */
#define PIO_IFDR 0x24u   /* input glitch filter disable */
#define PIO_SODR 0x30u   /* set output data */
#define PIO_CODR 0x34u   /* clear output data */
#define PIO_PDSR 0x3Cu   /* pin data status */
#define PIO_IDR  0x44u   /* interrupt disable */
#define PIO_MDDR 0x54u   /* multi-driver disable: push-pull, not open-drain */

#define PMC_PCER 0x10u
#define PMC_PCSR 0x18u

/* Peripheral IDs: PIOA/PIOB share 2, PIOC/PIOD share 3 */
#define AT91_PIO_PID(bank) ((bank) < 2 ? 2u : 3u)

static void at91_clk_enable(volatile uint32_t *pmc, int bank)
{
    uint32_t m = 1u << AT91_PIO_PID(bank);
    /* Reading PDSR needs the PIO clock running.  Linux normally leaves it on;
       enable it defensively rather than sample all-ones. */
    if (!(pmc[PMC_PCSR >> 2] & m))
        pmc[PMC_PCER >> 2] = m;
}

static void at91_pin(gpio_pin_t *p, volatile uint32_t *port, int bit)
{
    uint32_t m = 1u << bit;

    /* Remember pins we take from a peripheral so close() can hand them back */
    p->was_muxed = !(port[PIO_PSR >> 2] & m);

    port[PIO_IDR  >> 2] = m;   /* no interrupts */
    port[PIO_IFDR >> 2] = m;   /* no glitch filter — it would eat sampled bits */
    port[PIO_MDDR >> 2] = m;   /* push-pull: open-drain cannot drive SWCLK high */
    port[PIO_PER  >> 2] = m;   /* claim the pin for PIO */
    port[PIO_ODR  >> 2] = m;   /* input until the SWD layer drives it */

    p->set = &port[PIO_SODR >> 2];
    p->clr = &port[PIO_CODR >> 2];
    p->in  = &port[PIO_PDSR >> 2];
    p->oe  = &port[PIO_OER  >> 2];
    p->od  = &port[PIO_ODR  >> 2];
    p->dir_rmw = 0;
    p->bit  = bit;
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
    volatile uint32_t *base = map_phys(g, AT91_SYSC_PHYS, AT91_SYSC_SIZE);
    if (!base) return -1;

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
    if (p->was_muxed)
        p->port[PIO_PDR >> 2] = p->mask;
}

static void at91_close(gpio_t *g)
{
    at91_release(&g->clk);
    at91_release(&g->dio);
}

/* ------------------------------------------------------------------ */
/* SoC table                                                           */
/* ------------------------------------------------------------------ */

static const char *const mt7688_ids[] = { "mt7688", "mt7628", "ralink", NULL };
static const char *const at91_ids[]   = { "at91sam9", NULL };

static const struct gpio_soc socs[] = {
    { "mt7688",     mt7688_ids, 3, 0, "36",     "29",
      mt7688_open, mt7688_close },
    { "at91sam9x5", at91_ids,   4, 1, "PC12",   "PC11",
      at91_open,   at91_close },
};

#define NSOCS ((int)(sizeof(socs) / sizeof(socs[0])))

/* ------------------------------------------------------------------ */
/* Detection                                                           */
/* ------------------------------------------------------------------ */

/* /proc/device-tree/compatible is a list of NUL-separated strings; cpuinfo is
   plain text.  Both are read raw and lowercased for substring matching. */
static int file_has(const char *path, const char *const *needles)
{
    char buf[4096];
    int fd = open(path, O_RDONLY);
    if (fd < 0) return 0;
    ssize_t n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (n <= 0) return 0;

    for (ssize_t i = 0; i < n; i++) {
        if (buf[i] == '\0') buf[i] = '\n';
        else buf[i] = (char)tolower((unsigned char)buf[i]);
    }
    buf[n] = '\0';

    for (int i = 0; needles[i]; i++)
        if (strstr(buf, needles[i]))
            return 1;
    return 0;
}

static const struct gpio_soc *detect_soc(void)
{
    const char *env = getenv("SWD_SOC");
    if (env) {
        for (int i = 0; i < NSOCS; i++)
            if (strcasecmp(env, socs[i].name) == 0)
                return &socs[i];
        fprintf(stderr, "[!] Unknown SWD_SOC '%s'\n", env);
        return NULL;
    }

    for (int i = 0; i < NSOCS; i++) {
        if (file_has("/proc/device-tree/compatible", socs[i].ids) ||
            file_has("/proc/cpuinfo", socs[i].ids))
            return &socs[i];
    }

    fprintf(stderr, "[!] Could not detect host SoC — set SWD_SOC to one of:");
    for (int i = 0; i < NSOCS; i++)
        fprintf(stderr, " %s", socs[i].name);
    fprintf(stderr, "\n");
    return NULL;
}

/* ------------------------------------------------------------------ */
/* Pin names                                                           */
/* ------------------------------------------------------------------ */

/* "PC12", "pioC12", "gpio36", "36" -> bank + bit */
static int parse_pin(const char *s, int *bank, int *bit)
{
    static const char *const prefixes[] = { "gpio", "pio", "gp", "p", NULL };
    const char *p = s;

    while (isspace((unsigned char)*p)) p++;

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
        if (!isdigit((unsigned char)*p)) return -1;
        long v = strtol(p, NULL, 10);
        if (b < 0 || v < 0 || v > 31) return -1;
        *bank = b;
        *bit  = (int)v;
        return 0;
    }

    if (!isdigit((unsigned char)*p)) return -1;
    long v = strtol(p, NULL, 10);
    if (v < 0 || v > 1023) return -1;
    *bank = (int)(v / 32);
    *bit  = (int)(v % 32);
    return 0;
}

static int pin_from_env(const struct gpio_soc *soc, const char *env,
                        const char *def, int *bank, int *bit)
{
    const char *val = getenv(env);
    if (!val) val = def;

    if (parse_pin(val, bank, bit) != 0) {
        fprintf(stderr, "[!] Bad pin '%s' in %s\n", val, env);
        return -1;
    }
    if (*bank >= soc->nbanks) {
        fprintf(stderr, "[!] Pin '%s' out of range for %s (%d bank(s))\n",
                val, soc->name, soc->nbanks);
        return -1;
    }
    return 0;
}

static void pin_name(const struct gpio_soc *soc, int bank, int bit,
                     char *out, size_t len)
{
    if (soc->lettered)
        snprintf(out, len, "P%c%d", 'A' + bank, bit);
    else
        snprintf(out, len, "%d", bank * 32 + bit);
}

/* ------------------------------------------------------------------ */
/* Open / close                                                        */
/* ------------------------------------------------------------------ */

int gpio_open(gpio_t *g)
{
    memset(g, 0, sizeof(*g));

    const struct gpio_soc *soc = detect_soc();
    if (!soc) return -1;
    g->soc = soc;

    int cb, cbit, db, dbit;
    if (pin_from_env(soc, "SWD_SWCLK", soc->def_clk, &cb, &cbit) != 0) return -1;
    if (pin_from_env(soc, "SWD_SWDIO", soc->def_dio, &db, &dbit) != 0) return -1;

    if (cb == db && cbit == dbit) {
        fprintf(stderr, "[!] SWCLK and SWDIO are the same pin\n");
        return -1;
    }

    if (soc->open(g, cb, cbit, db, dbit) != 0) return -1;

    char cname[16], dname[16];
    pin_name(soc, cb, cbit, cname, sizeof(cname));
    pin_name(soc, db, dbit, dname, sizeof(dname));
    snprintf(g->desc, sizeof(g->desc), "%s SWCLK=%s SWDIO=%s",
             soc->name, cname, dname);
    return 0;
}

void gpio_close(gpio_t *g)
{
    if (!g->soc) return;
    g->soc->close(g);
    if (g->map) munmap(g->map, g->maplen);
    g->map = NULL;
    g->soc = NULL;
}

const char *gpio_desc(const gpio_t *g)
{
    return g->desc;
}
