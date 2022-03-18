#ifndef __AS3935_H__
#define __AS3935_H__
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

// Define Register_name {address, start_bit, end_bit}
#define AFE_GB {0x00, 1, 5}
#define PWD {0x00, 0, 0}
#define NF_LEV {0x01, 4, 6}
#define WDTH {0x01, 0, 3}
#define CL_STAT {0x02, 6, 6}
#define MIN_NUM_LIGH {0x02, 4, 5}
#define SREJ {0x02, 0, 3}
#define DISTANCE {0x07, 0, 5}
#define INT {0x03, 0, 3}

// Environment
#define SET_SENSOR_INDOOR(AS3935) as3935_set_aef_gain_boost(AS3935, AS3935_AEF_GAIN_BOOST_INDOOR)
#define SET_SENSOR_OUTDOOR(AS3935) as3935_set_aef_gain_boost(AS3935, AS3935_AEF_GAIN_BOOST_OUTDOOR)


//#define AS3935_SPI_TRANSFER_SPEED 2000000 // 2Mhz
#define AS3935_SPI_TRANSFER_SPEED 500000

struct _as3935 {
    int spi_fd;
    int ligh_num;
    float storm_distance;
};

typedef enum {
    AS3935_SPI_STATUS_OPEN_DEVICE_FAILED = -1,
    AS3935_SPI_STATUS_MODE_SET_FAILED = -2,
    AS3935_SPI_STATUS_BITS_PER_WORD_SET_FAILED = -3,
    AS3935_SPI_STATUS_MAXIMUN_SPEED_SET_FAILED = -4,
    AS3935_SPI_STATUS_SET_SUCCESS = 0
} AS3935_SPI_STATUS;

typedef enum {
    AS3935_POWER_STATUS_ON = 0,
    AS3935_POWER_STATUS_OFF = 1
} AS3935_POWER_STATUS;

typedef enum {
    AS3935_INT_EVENT_NOISE_LEVEL_TOO_HIGH = 0x01,
    AS3935_INT_EVENT_DISTURBER_DETECTED = 0x04,
    AS3935_INT_EVENT_LIGHTNING_INTERRUPT = 0x08
} AS3935_INT_EVENT;

typedef enum {
    AS3935_MIN_NUMBER_OF_LIGHTNING_ONE = 1,
    AS3935_MIN_NUMBER_OF_LIGHTNING_FIVE,
    AS3935_MIN_NUMBER_OF_LIGHTNING_NINE,
    AS3935_MIN_NUMBER_OF_LIGHTNING_SIXTEEN
} AS3935_MIN_NUMBER_OF_LIGHTNING;

typedef enum {
    AS3935_AEF_GAIN_BOOST_OUTDOOR = 0x0E,
    AS3935_AEF_GAIN_BOOST_INDOOR = 0x12
} AS3935_AEF_GAIN_BOOST;

typedef enum {
    AS3935_NOISE_FLOOR_LEVEL_390_28 = 0x00,
    AS3935_NOISE_FLOOR_LEVEL_630_45 = 0x01,
    AS3935_NOISE_FLOOR_LEVEL_860_62 = 0x02,
    AS3935_NOISE_FLOOR_LEVEL_1100_78 = 0x03,
    AS3935_NOISE_FLOOR_LEVEL_1140_95 = 0x04,
    AS3935_NOISE_FLOOR_LEVEL_1570_112 = 0x05,
    AS3935_NOISE_FLOOR_LEVEL_1800_130 = 0x06,
    AS3935_NOISE_FLOOR_LEVEL_2000_146 = 0x07
} AS3935_NOISE_FLOOR_LEVEL;
 
// Minimum Number of Lightning

AS3935_SPI_STATUS as3935_init(struct _as3935*, const char*);

bool as3935_power(struct _as3935*, AS3935_POWER_STATUS);

void as3935_config(struct _as3935*, uint8_t*, uint8_t*);

bool as3935_get_storm_distance(struct _as3935*, uint8_t*);

bool as3935_get_status(struct _as3935* as3935, uint8_t*);

bool as3935_set_min_num_lightning(struct _as3935*, uint8_t);

bool as3935_set_aef_gain_boost(struct _as3935*, AS3935_AEF_GAIN_BOOST);

bool as3935_set_noise_floor_level(struct _as3935*, AS3935_NOISE_FLOOR_LEVEL);

#endif