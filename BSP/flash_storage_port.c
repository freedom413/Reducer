#include "main.h"
#include "flash_storage.h"
#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"
#include <string.h>

static void flash_unlock(void)
{
    HAL_FLASH_Unlock();
}

static void flash_lock(void)
{
    HAL_FLASH_Lock();
}

static int flash_erase_page(uint32_t addr)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0;

    if (addr < FLASH_STORAGE_ADDR || addr >= FLASH_STORAGE_END_ADDR ||
        ((addr - FLASH_STORAGE_ADDR) % FLASH_STORAGE_PAGE_SIZE) != 0U) {
        return -1;
    }

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = FLASH_STORAGE_FIRST_PAGE +
                 (addr - FLASH_STORAGE_ADDR) / FLASH_STORAGE_PAGE_SIZE;
    erase.NbPages = 1;

    return (HAL_FLASHEx_Erase(&erase, &page_error) == HAL_OK) ? 0 : -1;
}

static int flash_program_dw(uint32_t addr, uint64_t data)
{
    if ((addr & 0x7U) != 0U ||
        addr < FLASH_STORAGE_ADDR ||
        addr > (FLASH_STORAGE_END_ADDR - sizeof(data))) {
        return -1;
    }

    return (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, data) == HAL_OK) ? 0 : -1;
}

static void flash_read(uint32_t addr, void *data, uint32_t len)
{
    memcpy(data, (const void *)addr, len);
}

static const flash_hw_ops_t flash_hw_ops = {
    .unlock = flash_unlock,
    .lock = flash_lock,
    .erase_page = flash_erase_page,
    .program_doubleword = flash_program_dw,
    .read = flash_read,
};

void flash_storage_register_user_ops(void)
{
    flash_storage_register_ops(&flash_hw_ops);
}
