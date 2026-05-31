#ifndef __ADS1256_CONFIG_H__
#define __ADS1256_CONFIG_H__

/*
 * Project-level ADS1256 selection.
 *
 * Keep this in one place so the board port and polling layer cannot drift.
 * Set ADS1256_ENABLE_B to 1 when the second converter is populated and its
 * DRDY/CS lines are connected.
 */
#define ADS1256_ENABLE_A  1
#define ADS1256_ENABLE_B  1

#endif /* __ADS1256_CONFIG_H__ */
