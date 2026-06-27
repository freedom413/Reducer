#include "flash_storage.h"
#include <stddef.h>
#include <string.h>

static const flash_hw_ops_t *flash_ops;

static uint32_t crc32_bytes(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    while (len-- > 0U) {
        crc ^= *data++;
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = (crc >> 1U) ^ ((crc & 1U) != 0U ? 0xEDB88320UL : 0U);
        }
    }
    return ~crc;
}

static bool config_valid(const persistent_config_t *config)
{
    if (config->magic != PERSISTENT_CONFIG_MAGIC ||
        config->version != PERSISTENT_CONFIG_VERSION ||
        config->length != sizeof(*config)) {
        return false;
    }
    return config->crc32 ==
           crc32_bytes((const uint8_t *)config,
                       (uint32_t)offsetof(persistent_config_t, crc32));
}

static int find_latest(persistent_config_t *latest, uint32_t *next_addr)
{
    bool found = false;
    persistent_config_t candidate;
    uint32_t address = FLASH_STORAGE_ADDR;
    uint32_t best_sequence = 0U;

    if (flash_ops == NULL || flash_ops->read == NULL) {
        return -1;
    }
    while (address < FLASH_STORAGE_ADDR + FLASH_STORAGE_PAGE_SIZE) {
        flash_ops->read(address, &candidate, sizeof(candidate));
        if (candidate.magic == 0xFFFFFFFFUL) {
            break;
        }
        if (config_valid(&candidate) &&
            (!found || candidate.sequence >= best_sequence)) {
            *latest = candidate;
            best_sequence = candidate.sequence;
            found = true;
        }
        address += FLASH_STORAGE_RECORD_SIZE;
    }
    if (next_addr != NULL) {
        *next_addr = address;
    }
    return found ? 0 : -1;
}

void flash_storage_register_ops(const flash_hw_ops_t *ops)
{
    flash_ops = ops;
}

int flash_storage_init(void)
{
    return flash_ops == NULL ? -1 : 0;
}

void flash_storage_config_defaults(persistent_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->magic = PERSISTENT_CONFIG_MAGIC;
    config->version = PERSISTENT_CONFIG_VERSION;
    config->length = sizeof(*config);
    config->vref_uv = PERSISTENT_CONFIG_DEFAULT_VREF_UV;
    config->sample_rate_x10 = PERSISTENT_CONFIG_DEFAULT_SAMPLE_RATE_X10;
    config->channel_mask = PERSISTENT_CONFIG_DEFAULT_CHANNEL_MASK;
    config->pga_gain = PERSISTENT_CONFIG_DEFAULT_PGA;
    config->filter_length = PERSISTENT_CONFIG_DEFAULT_FILTER_LENGTH;
    config->telemetry_mode = PERSISTENT_CONFIG_DEFAULT_TELEMETRY_MODE;
}

bool flash_storage_is_valid(void)
{
    persistent_config_t config;
    return find_latest(&config, NULL) == 0;
}

int flash_storage_load_config(persistent_config_t *config)
{
    if (config == NULL) {
        return -1;
    }
    return find_latest(config, NULL);
}

int flash_storage_save_config(persistent_config_t *config)
{
    persistent_config_t latest;
    persistent_config_t verified;
    uint32_t address;
    int ret;

    if (config == NULL || flash_ops == NULL || flash_ops->unlock == NULL ||
        flash_ops->lock == NULL || flash_ops->erase_page == NULL ||
        flash_ops->program_doubleword == NULL || flash_ops->read == NULL) {
        return -1;
    }

    if (find_latest(&latest, &address) == 0) {
        config->sequence = latest.sequence + 1U;
    } else {
        config->sequence = 1U;
    }
    config->magic = PERSISTENT_CONFIG_MAGIC;
    config->version = PERSISTENT_CONFIG_VERSION;
    config->length = sizeof(*config);
    config->crc32 = crc32_bytes((const uint8_t *)config,
                                (uint32_t)offsetof(persistent_config_t, crc32));

    flash_ops->unlock();
    if (address >= FLASH_STORAGE_ADDR + FLASH_STORAGE_PAGE_SIZE) {
        ret = flash_ops->erase_page(FLASH_STORAGE_ADDR);
        address = FLASH_STORAGE_ADDR;
        if (ret != 0) {
            flash_ops->lock();
            return ret;
        }
    }
    for (uint32_t offset = 0U; offset < sizeof(*config); offset += 8U) {
        uint64_t dword;
        memcpy(&dword, ((const uint8_t *)config) + offset, sizeof(dword));
        if (flash_ops->program_doubleword(address + offset, dword) != 0) {
            flash_ops->lock();
            return -2;
        }
    }
    flash_ops->lock();
    flash_ops->read(address, &verified, sizeof(verified));
    if (!config_valid(&verified) ||
        memcmp(&verified, config, sizeof(verified)) != 0) {
        return -3;
    }
    return 0;
}

int flash_storage_save_zero(const int32_t *offset)
{
    persistent_config_t config;
    if (offset == NULL) {
        return -1;
    }
    if (flash_storage_load_config(&config) != 0) {
        flash_storage_config_defaults(&config);
    }
    memcpy(config.zero_offset, offset, sizeof(config.zero_offset));
    config.flags |= PERSISTENT_CONFIG_FLAG_ZERO_VALID;
    return flash_storage_save_config(&config);
}

int flash_storage_load_zero(int32_t *offset)
{
    persistent_config_t config;
    if (offset == NULL || flash_storage_load_config(&config) != 0 ||
        (config.flags & PERSISTENT_CONFIG_FLAG_ZERO_VALID) == 0U) {
        if (offset != NULL) {
            memset(offset, 0, sizeof(config.zero_offset));
        }
        return -1;
    }
    memcpy(offset, config.zero_offset, sizeof(config.zero_offset));
    return 0;
}

int flash_storage_clear(void)
{
    if (flash_ops == NULL || flash_ops->unlock == NULL ||
        flash_ops->lock == NULL || flash_ops->erase_page == NULL) {
        return -1;
    }
    flash_ops->unlock();
    int ret = flash_ops->erase_page(FLASH_STORAGE_ADDR);
    flash_ops->lock();
    return ret;
}
