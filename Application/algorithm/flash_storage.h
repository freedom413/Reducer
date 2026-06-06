#ifndef __FLASH_STORAGE_H__
#define __FLASH_STORAGE_H__

#include <stdbool.h>
#include <stdint.h>

#define FLASH_STORAGE_PAGE          63U
#define FLASH_STORAGE_ADDR          0x0801F800U
#define FLASH_STORAGE_PAGE_SIZE     2048U
#define FLASH_STORAGE_RECORD_SIZE   64U
#define FLASH_STORAGE_CHANNEL_COUNT 8U

#define PERSISTENT_CONFIG_MAGIC      0x52444346UL
#define PERSISTENT_CONFIG_VERSION    0x0003U
#define PERSISTENT_CONFIG_FLAG_ZERO_VALID 0x01U

#define PERSISTENT_CONFIG_DEFAULT_VREF_UV       2500000UL
#define PERSISTENT_CONFIG_DEFAULT_SAMPLE_RATE_X10 1000UL
#define PERSISTENT_CONFIG_DEFAULT_CHANNEL_MASK  0x00FFU
#define PERSISTENT_CONFIG_DEFAULT_PGA           16U
#define PERSISTENT_CONFIG_DEFAULT_FILTER_LENGTH 16U
#define PERSISTENT_CONFIG_DEFAULT_TELEMETRY_MODE 0U

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t length;
    uint32_t sequence;
    uint32_t vref_uv;
    uint32_t sample_rate_x10;
    int32_t zero_offset[FLASH_STORAGE_CHANNEL_COUNT];
    uint16_t channel_mask;
    uint8_t pga_gain;
    uint8_t filter_length;
    uint8_t telemetry_mode;
    uint8_t flags;
    uint8_t reserved[2];
    uint32_t crc32;
} persistent_config_t;

_Static_assert(sizeof(persistent_config_t) == FLASH_STORAGE_RECORD_SIZE,
               "persistent_config_t must be one flash record");

typedef struct {
    void (*unlock)(void);
    void (*lock)(void);
    int (*erase_page)(uint32_t page_address);
    int (*program_doubleword)(uint32_t address, uint64_t data);
    void (*read)(uint32_t address, void *data, uint32_t len);
} flash_hw_ops_t;

void flash_storage_register_ops(const flash_hw_ops_t *ops);
void flash_storage_register_user_ops(void);
int flash_storage_init(void);
void flash_storage_config_defaults(persistent_config_t *config);
bool flash_storage_is_valid(void);
int flash_storage_load_config(persistent_config_t *config);
int flash_storage_save_config(persistent_config_t *config);
int flash_storage_save_zero(const int32_t *offset);
int flash_storage_load_zero(int32_t *offset);
int flash_storage_clear(void);

#endif
