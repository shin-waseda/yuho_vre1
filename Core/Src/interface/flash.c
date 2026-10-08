#include "interface/flash.h"

#include <string.h>
#include "main.h"

#define FLASH_USER_SECTOR  FLASH_SECTOR_11
#define FLASH_USER_ADDR    0x080E0000u
#define FLASH_USER_SIZE    (128u * 1024u)

#define FLASH_USER_MAGIC   0x50414D59u // "YMAP"

// セクタの先頭に置く見出し。データはこの直後に続く。
typedef struct {
    uint32_t magic;
    uint32_t len;
    uint32_t sum;   // データの各バイトの合計(壊れていないかの確かめ)
    uint32_t reserved;
} FlashUserHeader;

static uint32_t Checksum(const uint8_t *p, uint32_t len) {
    uint32_t s = 0;
    for (uint32_t i = 0; i < len; i++) s += p[i];
    return s;
}

static bool ProgramWords(uint32_t addr, const uint8_t *src, uint32_t len) {
    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t w = 0xFFFFFFFFu;
        uint32_t n = (len - i < 4) ? (len - i) : 4;
        memcpy(&w, &src[i], n);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i, w) != HAL_OK) return false;
    }
    return true;
}

bool Flash_WriteUserData(const void *data, uint32_t len) {
    if (len + sizeof(FlashUserHeader) > FLASH_USER_SIZE) return false;

    FlashUserHeader h = {
        .magic = FLASH_USER_MAGIC,
        .len = len,
        .sum = Checksum((const uint8_t *)data, len),
        .reserved = 0xFFFFFFFFu,
    };

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Sector = FLASH_USER_SECTOR,
        .NbSectors = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3, // 2.7〜3.6V(32bit 単位で書く)
    };
    uint32_t bad_sector = 0;
    bool ok = (HAL_FLASHEx_Erase(&erase, &bad_sector) == HAL_OK)
           && ProgramWords(FLASH_USER_ADDR, (const uint8_t *)&h, sizeof(h))
           && ProgramWords(FLASH_USER_ADDR + sizeof(h), (const uint8_t *)data, len);

    HAL_FLASH_Lock();

    // 読み直して確かめる
    return ok && memcmp((const void *)(FLASH_USER_ADDR + sizeof(h)), data, len) == 0;
}

bool Flash_ReadUserData(void *data, uint32_t len) {
    const FlashUserHeader *h = (const FlashUserHeader *)FLASH_USER_ADDR;
    if (h->magic != FLASH_USER_MAGIC || h->len != len) return false;
    const uint8_t *src = (const uint8_t *)(FLASH_USER_ADDR + sizeof(FlashUserHeader));
    if (Checksum(src, len) != h->sum) return false;
    memcpy(data, src, len);
    return true;
}
