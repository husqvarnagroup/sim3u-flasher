#include "sim3u_flash.h"
#include "swd.h"

#include <stdio.h>
#include <string.h>

static int flash_busy_wait(swd_ctx_t *ctx)
{
    for (int i = 0; i < SIM3U_FLASH_BUSY_TIMEOUT; i++) {
        uint32_t cfg;
        if (swd_mem_read32(ctx, SIM3U_FLASH_CONFIG_ALL, &cfg) != 0) return -1;
        if (!(cfg & SIM3U_FLASH_CONFIG_BUSYF)) return 0;
    }
    fprintf(stderr, "[!] Flash busy timeout\n");
    return -1;
}

int sim3u_init(swd_ctx_t *ctx)
{
    /* Disable watchdog */
    if (swd_mem_write32(ctx, SIM3U_WDTIMER0_WDTKEY, SIM3U_WDTIMER0_KEY1) != 0) return -1;
    if (swd_mem_write32(ctx, SIM3U_WDTIMER0_WDTKEY, SIM3U_WDTIMER0_KEY2) != 0) return -1;
    if (swd_mem_write32(ctx, SIM3U_WDTIMER0_CONTROL_SET, SIM3U_WDTIMER0_DBGMD) != 0) return -1;

    /* Enable flash controller clock */
    if (swd_mem_write32(ctx, SIM3U_CLKCTRL0_APBCLKG0_SET, SIM3U_CLKCTRL0_FLCTRLCEN) != 0) return -1;

    /* Ensure erase mode is cleared */
    if (swd_mem_write32(ctx, SIM3U_FLASH_CONFIG_CLR, SIM3U_FLASH_CONFIG_ERASEEN) != 0) return -1;

    return 0;
}

static int erase_page(swd_ctx_t *ctx, uint32_t addr)
{
    int ret = -1;

    if (flash_busy_wait(ctx) != 0) return -1;

    if (swd_mem_write32(ctx, SIM3U_FLASH_CONFIG_SET, SIM3U_FLASH_CONFIG_ERASEEN) != 0) goto out;
    if (swd_mem_write32(ctx, SIM3U_FLASH_WRADDR, addr) != 0) goto out;
    if (swd_mem_write32(ctx, SIM3U_FLASH_KEY, SIM3U_FLASH_KEY_INITIAL) != 0) goto out;
    if (swd_mem_write32(ctx, SIM3U_FLASH_KEY, SIM3U_FLASH_KEY_SINGLE) != 0) goto out;
    /* Writing any value to WRDATA triggers the erase */
    if (swd_mem_write32(ctx, SIM3U_FLASH_WRDATA, 0x00000000u) != 0) goto out;
    if (flash_busy_wait(ctx) != 0) goto out;

    ret = 0;

out:
    /* On failure the key may still be armed from KEY_INITIAL/KEY_SINGLE;
       re-lock so a stray WRDATA write cannot trigger an erase */
    if (ret != 0)
        swd_mem_write32(ctx, SIM3U_FLASH_KEY, SIM3U_FLASH_KEY_LOCK);
    swd_mem_write32(ctx, SIM3U_FLASH_CONFIG_CLR, SIM3U_FLASH_CONFIG_ERASEEN);
    return ret;
}

int sim3u_erase(swd_ctx_t *ctx, uint32_t size)
{
    uint32_t num_pages = (size + SIM3U_FLASH_PAGE_SIZE - 1) / SIM3U_FLASH_PAGE_SIZE;
    printf("[*] Erasing %u page(s)...\n", num_pages);

    for (uint32_t page = 0; page < num_pages; page++) {
        uint32_t addr = SIM3U_FLASH_BASE + page * SIM3U_FLASH_PAGE_SIZE;
        if (erase_page(ctx, addr) != 0) {
            fprintf(stderr, "[!] Erase failed at page %u (0x%08x)\n", page, addr);
            return -1;
        }
        if ((page & 0xF) == 0 || page == num_pages - 1) {
            printf("  erased %u / %u\r", page + 1, num_pages);
            fflush(stdout);
        }
    }

    printf("\n[+] Erase complete.\n");
    return 0;
}

int sim3u_write(swd_ctx_t *ctx, const uint8_t *data, uint32_t size)
{
    uint32_t hw_count = (size + 1) / 2;
    printf("[*] Writing %u bytes (%u halfwords)...\n", size, hw_count);

    if (flash_busy_wait(ctx) != 0) return -1;

    /* Arm for multiple sequential writes */
    if (swd_mem_write32(ctx, SIM3U_FLASH_KEY, SIM3U_FLASH_KEY_INITIAL) != 0) return -1;
    if (swd_mem_write32(ctx, SIM3U_FLASH_KEY, SIM3U_FLASH_KEY_MULTIPLE) != 0) return -1;

    int ret = 0;
    for (uint32_t i = 0; i < hw_count; i++) {
        uint16_t hw;
        if (2 * i + 1 < size)
            hw = (uint16_t)(data[2 * i]) | ((uint16_t)(data[2 * i + 1]) << 8);
        else
            hw = (uint16_t)(data[2 * i]) | 0xFF00u;

        if (flash_busy_wait(ctx) != 0) { ret = -1; break; }

        uint32_t target_addr = SIM3U_FLASH_BASE + i * 2;
        if (swd_mem_write32(ctx, SIM3U_FLASH_WRADDR, target_addr) != 0) { ret = -1; break; }
        if (swd_mem_write32(ctx, SIM3U_FLASH_WRDATA, hw) != 0) { ret = -1; break; }

        if ((i & 0xFF) == 0xFF)
            printf("  written %u / %u bytes\r", (i + 1) * 2, size);
    }

    if (ret == 0) ret = flash_busy_wait(ctx);

    /* Always lock; a failed lock write leaves flash writable, so report it */
    int lock_ret = swd_mem_write32(ctx, SIM3U_FLASH_KEY, SIM3U_FLASH_KEY_LOCK);
    if (ret == 0 && lock_ret != 0) {
        fprintf(stderr, "[!] Flash re-lock failed\n");
        ret = -1;
    }

    if (ret != 0) {
        fprintf(stderr, "[!] Write failed\n");
        return -1;
    }

    printf("\n[+] Write complete.\n");
    return 0;
}

int sim3u_verify(swd_ctx_t *ctx, const uint8_t *data, uint32_t size)
{
    enum { VERIFY_CHUNK_WORDS = 256 };  /* 1 KB per block read */
    uint32_t words[VERIFY_CHUNK_WORDS];

    printf("[*] Verifying %u bytes...\n", size);

    for (uint32_t off = 0; off < size; off += sizeof(words)) {
        uint32_t bytes = size - off;
        if (bytes > sizeof(words)) bytes = sizeof(words);
        uint32_t count = (bytes + 3) / 4;

        if (swd_mem_read_block(ctx, SIM3U_FLASH_BASE + off, words, count) != 0)
            return -1;

        for (uint32_t w = 0; w < count; w++) {
            uint32_t i = off + w * 4;
            uint32_t expected = 0xFFFFFFFFu;
            uint32_t n = (size - i >= 4) ? 4 : (size - i);
            memcpy(&expected, data + i, n);

            if (words[w] != expected) {
                fprintf(stderr, "[!] Verify mismatch at 0x%08x: got 0x%08x expected 0x%08x\n",
                        SIM3U_FLASH_BASE + i, words[w], expected);
                return -1;
            }
        }
    }

    printf("[+] Verification passed.\n");
    return 0;
}
