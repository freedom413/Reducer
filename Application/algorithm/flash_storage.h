#ifndef __FLASH_STORAGE_H__
#define __FLASH_STORAGE_H__

#include <stdint.h>

// ============================================================================
// Flash Storage Configuration
// ============================================================================
// STM32G431CB: 128KB Flash, 2KB pages
// Using last page (page 127) at 0x0801F800 for calibration data

#define FLASH_STORAGE_PAGE     127
#define FLASH_STORAGE_ADDR     0x0801F800
#define FLASH_STORAGE_SIZE      32  // bytes reserved

// Calibration data version
#define CALIB_DATA_VERSION     0x0001

// ============================================================================
// Calibration Data Structure (stored in Flash)
// ============================================================================
typedef struct __attribute__((packed)) {
    uint16_t version;           // Data structure version
    int32_t zero_offset[6];     // Zero offset for each channel
    uint16_t crc16;             // CRC-16 of zero_offset data
    uint16_t magic;             // Magic number for validation
} calib_data_t;

#define CALIB_MAGIC             0xCA1B  // Calibration valid marker

// ============================================================================
// Flash Hardware Abstraction Interface (function pointers)
// ============================================================================
// Users must implement these functions and register them before use

typedef struct {
    /** @brief Unlock flash for write operation */
    void (*unlock)(void);

    /** @brief Lock flash after write operation */
    void (*lock)(void);

    /** @brief Erase a flash page
     * @param page_address Start address of the page
     * @return 0 on success, negative on error
     */
    int (*erase_page)(uint32_t page_address);

    /** @brief Program a 64-bit double-word to flash
     * @param address Target address (must be 8-byte aligned)
     * @param data Data to program
     * @return 0 on success, negative on error
     */
    int (*program_doubleword)(uint32_t address, uint64_t data);

    /** @brief Read data from flash
     * @param address Source address
     * @param data Buffer to store read data
     * @param len Number of bytes to read
     */
    void (*read)(uint32_t address, void *data, uint32_t len);
} flash_hw_ops_t;

// ============================================================================
// Flash Storage API
// ============================================================================

/**
 * @brief Register flash hardware operations (must be called before init)
 * @param ops Pointer to flash hardware operations structure
 */
void flash_storage_register_ops(const flash_hw_ops_t *ops);

/**
 * @brief Initialize flash storage
 * @return 0 on success, negative on error
 */
int flash_storage_init(void);

/**
 * @brief Check if calibration data is valid
 * @return true if valid calibration exists
 */
bool flash_storage_is_valid(void);

/**
 * @brief Save zero offset to flash
 * @param offset Array of 6 channel zero offsets
 * @return 0 on success, negative on error
 */
int flash_storage_save_zero(const int32_t *offset);

/**
 * @brief Load zero offset from flash
 * @param offset Array to store 6 channel zero offsets
 * @return 0 on success, negative on error
 */
int flash_storage_load_zero(int32_t *offset);

/**
 * @brief Clear calibration data
 * @return 0 on success, negative on error
 */
int flash_storage_clear(void);

#endif // __FLASH_STORAGE_H__
