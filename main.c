#include <errno.h>
#include <getopt.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "swd.h"
#include "sim3u_flash.h"

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [options] <firmware.bin>\n"
        "  -e       erase only, do not write\n"
        "  -v       verify only, do not write\n"
        "  -n       skip verify after write\n"
        "  -k       keep CPU halted after done\n"
        "  -d       debug: dump IDCODE and exit\n"
        "\n"
        "Env:\n"
        "  SWD_DELAY=N   extra palmbus dummy reads per clock half-period (default 0)\n",
        prog);
}

static uint8_t *load_file(const char *path, uint32_t *size_out)
{
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return NULL; }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    rewind(f);
    if (sz <= 0 || sz > (long)SIM3U_FLASH_SIZE_256K) {
        fprintf(stderr, "[!] File size %ld out of range\n", sz);
        fclose(f);
        return NULL;
    }
    uint8_t *buf = malloc((size_t)sz);
    if (!buf) { fclose(f); return NULL; }
    if ((long)fread(buf, 1, (size_t)sz, f) != sz) {
        perror("fread"); free(buf); fclose(f); return NULL;
    }
    fclose(f);
    *size_out = (uint32_t)sz;
    return buf;
}

int main(int argc, char **argv)
{
    int opt_erase_only  = 0;
    int opt_verify_only = 0;
    int opt_no_verify   = 0;
    int opt_keep_halted = 0;
    int opt_debug       = 0;

    int c;
    while ((c = getopt(argc, argv, "evnkd")) != -1) {
        switch (c) {
        case 'e': opt_erase_only  = 1; break;
        case 'v': opt_verify_only = 1; break;
        case 'n': opt_no_verify   = 1; break;
        case 'k': opt_keep_halted = 1; break;
        case 'd': opt_debug       = 1; break;
        default:  usage(argv[0]); return 1;
        }
    }

    if (optind >= argc && !opt_debug) {
        usage(argv[0]);
        return 1;
    }

    uint8_t  *fw   = NULL;
    uint32_t  size = 0;

    if (optind < argc) {
        fw = load_file(argv[optind], &size);
        if (!fw) return 1;
        printf("[+] Loaded %u bytes from %s\n", size, argv[optind]);
    }

    swd_ctx_t *ctx = swd_open();
    if (!ctx) { free(fw); return 1; }

    uint32_t idcode = 0;
    if (swd_connect(ctx, &idcode) != 0) {
        fprintf(stderr, "[!] SWD connect failed\n");
        swd_close(ctx); free(fw); return 1;
    }
    printf("[+] IDCODE: 0x%08x\n", idcode);

    if (idcode != SIM3U_EXPECTED_IDCODE) {
        fprintf(stderr, "[!] Unexpected IDCODE (expected 0x%08x)\n", SIM3U_EXPECTED_IDCODE);
        swd_close(ctx); free(fw); return 1;
    }

    if (opt_debug) {
        swd_close(ctx); free(fw); return 0;
    }

    printf("[*] Halting CPU...\n");
    if (swd_halt(ctx) != 0) {
        fprintf(stderr, "[!] CPU halt failed\n");
        swd_close(ctx); free(fw); return 1;
    }
    printf("[+] CPU halted.\n");

    if (sim3u_init(ctx) != 0) {
        fprintf(stderr, "[!] Device init failed\n");
        swd_close(ctx); free(fw); return 1;
    }

    int ret = 0;

    if (opt_verify_only) {
        ret = sim3u_verify(ctx, fw, size);
    } else {
        ret = sim3u_erase(ctx, size);
        if (ret == 0 && !opt_erase_only) {
            ret = sim3u_write(ctx, fw, size);
            if (ret == 0 && !opt_no_verify)
                ret = sim3u_verify(ctx, fw, size);
        }
    }

    if (!opt_keep_halted) {
        printf("[*] Resuming CPU...\n");
        if (swd_run(ctx) != 0)
            fprintf(stderr, "[!] CPU resume failed\n");
        else
            printf("[+] CPU running.\n");
    }

    swd_close(ctx);
    free(fw);

    if (ret == 0) printf("[+] Done.\n");
    return ret ? 1 : 0;
}
