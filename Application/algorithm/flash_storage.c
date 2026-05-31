#include "flash_storage.h"
#include <string.h>

// ============================================================================
// CRC-16-CCITT implementation for data validation
// ============================================================================
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// ============================================================================
// Flash hardware operations (function pointers)
// ============================================================================
static const flash_hw_ops_t *flash_ops = NULL;

void flash_storage_register_ops(const flash_hw_ops_t *ops)
{
    flash_ops = ops;
}

// ============================================================================
// Public API Implementation
// ============================================================================

int flash_storage_init(void)
{
    return 0;  // No initialization needed for Flash storage
}

bool flash_storage_is_valid(void)
{
    calib_data_t data;
    uint16_t calculated_crc;

    if (flash_ops == NULL || flash_ops->read == NULL) {
        return false;
    }

    // Read the entire structure from Flash
    flash_ops->read(FLASH_STORAGE_ADDR, &data, sizeof(calib_data_t));

    // Check magic number
    if (data.magic != CALIB_MAGIC) {
        return false;
    }

    // Check version
    if (data.version != CALIB_DATA_VERSION) {
        return false;
    }

    // Verify CRC
    calculated_crc = crc16_ccitt((const uint8_t *)data.zero_offset,
                                  sizeof(data.zero_offset));
    if (calculated_crc != data.crc16) {
        return false;
    }

    return true;
}

int flash_storage_save_zero(const int32_t *offset)
{
    calib_data_t data;
    uint32_t flash_addr = FLASH_STORAGE_ADDR;
    uint32_t total_dwords;
    int ret;

    if (offset == NULL ||
        flash_ops == NULL || flash_ops->unlock == NULL ||
        flash_ops->lock == NULL || flash_ops->erase_page == NULL ||
        flash_ops->program_doubleword == NULL || flash_ops->read == NULL) {
        return -1;  // Ops not registered
    }

    memset(&data, 0, sizeof(data));

    // Prepare calibration data
    data.version = CALIB_DATA_VERSION;
    memcpy(data.zero_offset, offset, sizeof(data.zero_offset));
    data.magic = CALIB_MAGIC;
    data.crc16 = crc16_ccitt((const uint8_t *)data.zero_offset,
                              sizeof(data.zero_offset));

    // Unlock Flash
    flash_ops->unlock();

    // Erase the page
    ret = flash_ops->erase_page(FLASH_STORAGE_ADDR);
    if (ret != 0) {
        flash_ops->lock();
        return ret;
    }

    // Program double-words (8 bytes at a time).
    // Copy through a local uint64_t to avoid reading past the packed struct.
    total_dwords = (sizeof(calib_data_t) + 7) / 8;

    for (uint32_t i = 0; i < total_dwords; i++) {
        uint64_t dword = 0ULL;
        memcpy(&dword, ((const uint8_t *)&data) + (i * 8U), sizeof(dword));
        ret = flash_ops->program_doubleword(flash_addr, dword);
        if (ret != 0) {
            flash_ops->lock();
            return -3;
        }
        flash_addr += 8;
    }

    // Lock Flash
    flash_ops->lock();

    return flash_storage_is_valid() ? 0 : -4;
}

int flash_storage_load_zero(int32_t *offset)
{
    calib_data_t data;

    if (offset == NULL) {
        return -1;
    }

    if (!flash_storage_is_valid()) {
        // Return zeros if no valid calibration
        memset(offset, 0, sizeof(int32_t) * 6);
        return -1;
    }

    if (flash_ops == NULL || flash_ops->read == NULL) {
        return -1;
    }

    // Read from Flash
    flash_ops->read(FLASH_STORAGE_ADDR, &data, sizeof(calib_data_t));
    memcpy(offset, data.zero_offset, sizeof(int32_t) * 6);

    return 0;
}

int flash_storage_clear(void)
{
    int ret;

    if (flash_ops == NULL || flash_ops->unlock == NULL ||
        flash_ops->lock == NULL || flash_ops->erase_page == NULL) {
        return -1;
    }

    flash_ops->unlock();
    ret = flash_ops->erase_page(FLASH_STORAGE_ADDR);
    flash_ops->lock();

    return ret;
}
