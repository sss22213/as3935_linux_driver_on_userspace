#include "as3935.h"

static inline bool _reg_write(struct _as3935* as3935, uint8_t reg_address, uint8_t value)
{
    uint8_t spi_tx_data_byte[2] = {0};

    reg_address &= ~(1 << 6 | 1 << 7);

    spi_tx_data_byte[0] = reg_address;
    spi_tx_data_byte[1] = value ;

    struct spi_ioc_transfer spi_trans = {
        .tx_buf = (unsigned long)(spi_tx_data_byte),
        .rx_buf = 0,
        .len = 2,
        .delay_usecs = 1,
        .speed_hz = AS3935_SPI_TRANSFER_SPEED,
        .bits_per_word = 8,
    };

    return (ioctl(as3935->spi_fd, SPI_IOC_MESSAGE(1), &spi_trans)==-1)?false:true;
}       

static inline bool _reg_read(struct _as3935* as3935, uint8_t reg_address, uint8_t *read_value)
{
    uint8_t spi_tx_data[2] = {0};
    uint8_t spi_rx_data[2] = {0};

    bool result = true;

    memset(read_value, 0, sizeof(uint8_t));

    spi_tx_data[0] |= (1 << 6 | reg_address);
    
    struct spi_ioc_transfer spi_trans = {
        .tx_buf = (unsigned long)(spi_tx_data),
        .rx_buf = (unsigned long)(spi_rx_data),
        .len = 2,
        .delay_usecs = 1,
        .speed_hz = AS3935_SPI_TRANSFER_SPEED,
        .bits_per_word = 8,
    };

    if (ioctl(as3935->spi_fd, SPI_IOC_MESSAGE(1), &spi_trans)==-1) {
        result = false;
    } else {
        result = true;
    }

    *read_value = spi_rx_data[1];

    return result;
}

AS3935_SPI_STATUS as3935_init(struct _as3935* as3935, const char *spi_path)
{
    int ret = 0;

    uint32_t mode = SPI_MODE_1;

    uint8_t bit = 8;

    uint32_t speed = AS3935_SPI_TRANSFER_SPEED;

    AS3935_SPI_STATUS as3935_spi_status = AS3935_SPI_STATUS_SET_SUCCESS;

    // Turn on spi
    as3935->spi_fd = open(spi_path, O_RDWR);
    if (as3935->spi_fd < 0) {
        as3935_spi_status = AS3935_SPI_STATUS_OPEN_DEVICE_FAILED;
        goto as3935_spi_init_result;
    }

    // Config WR_MODE must before SPI_IOC_RD_MODE.
    
    ret = ioctl(as3935->spi_fd, SPI_IOC_WR_MODE, &mode);
    if (ret == -1) {
        as3935_spi_status = AS3935_SPI_STATUS_MODE_SET_FAILED;
        goto as3935_spi_init_result;
    }

    ret = ioctl(as3935->spi_fd, SPI_IOC_RD_MODE, &mode);
    if (ret == -1) {
        as3935_spi_status = AS3935_SPI_STATUS_MODE_SET_FAILED;
        goto as3935_spi_init_result;
    }

    // 8 bits per word
    ret = ioctl(as3935->spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bit);
    if (ret == -1) {
        as3935_spi_status = AS3935_SPI_STATUS_BITS_PER_WORD_SET_FAILED;
        goto as3935_spi_init_result;
    }

    ret = ioctl(as3935->spi_fd, SPI_IOC_RD_BITS_PER_WORD, &bit);
    if(ret == -1) {
        as3935_spi_status = AS3935_SPI_STATUS_BITS_PER_WORD_SET_FAILED;
        goto as3935_spi_init_result;
    }

    // The as3935 speed limit is 2Mhz 
    ret = ioctl(as3935->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if(ret == -1) {
        as3935_spi_status = AS3935_SPI_STATUS_MAXIMUN_SPEED_SET_FAILED;
        goto as3935_spi_init_result;
    }

    ret = ioctl(as3935->spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if(ret == -1) {
        as3935_spi_status = AS3935_SPI_STATUS_MAXIMUN_SPEED_SET_FAILED;
        goto as3935_spi_init_result;
    }

as3935_spi_init_result:
    return as3935_spi_status;
}

static inline bool _set_reg_bit(struct _as3935* as3935, uint8_t reg_address, uint8_t start_bit, uint8_t end_bit, uint8_t value)
{
    uint8_t mask = 0;

    uint8_t read_buff = 0xFF;

    bool result = true;

    for (int idx = 0; idx < 8 ; idx++) {
        if (idx >= start_bit && idx <= end_bit) {
            mask |= 1 << idx;
        }
    }

    result &= _reg_read(as3935, reg_address, &read_buff);

    read_buff &= ~(mask);
    read_buff |= (value << start_bit);

    result &= _reg_write(as3935, reg_address, read_buff);

    return result;

}

static inline bool _get_reg_bit(struct _as3935* as3935, uint8_t reg_address, uint8_t start_bit, uint8_t end_bit, uint8_t *value)
{
    uint8_t mask = 0;

    bool result = true;

    for (int idx = start_bit; idx <= end_bit ; idx++) {
        mask |= (1 << (idx-start_bit));
    }

    result = _reg_read(as3935, reg_address, value);

    *value = (*value >> start_bit);

    *value &= mask;

    return result;
}

bool as3935_power(struct _as3935* as3935, AS3935_POWER_STATUS power_status)
{
    uint8_t pwd[] = PWD;

    return _set_reg_bit(as3935, pwd[0], pwd[1], pwd[2], power_status);
}

bool as3935_set_min_num_lightning(struct _as3935* as3935, uint8_t value)
{
    uint8_t min_num_lightning[] = MIN_NUM_LIGH;

    return _set_reg_bit(as3935, min_num_lightning[0], min_num_lightning[1], min_num_lightning[2], value);
}

bool as3935_get_storm_distance(struct _as3935* as3935, uint8_t *value)
{
    uint8_t distance[] = DISTANCE;

    return _get_reg_bit(as3935, distance[0], distance[1], distance[2], value);
}

bool as3935_get_status(struct _as3935* as3935, uint8_t *value)
{
    uint8_t int_status[] = INT;

    return _get_reg_bit(as3935, int_status[0], int_status[1], int_status[2], value);
}

bool as3935_set_aef_gain_boost(struct _as3935* as3935, AS3935_AEF_GAIN_BOOST as3935_aef_gain_boost)
{
    uint8_t aef_gb[] = AFE_GB;

    return _set_reg_bit(as3935, aef_gb[0], aef_gb[1], aef_gb[2], as3935_aef_gain_boost);
}

bool as3935_set_noise_floor_level(struct _as3935* as3935, AS3935_NOISE_FLOOR_LEVEL as3935_noise_floor_level)
{
    uint8_t nosie_flooe_level[] = NF_LEV;

    return _set_reg_bit(as3935, nosie_flooe_level[0], nosie_flooe_level[1], nosie_flooe_level[2], as3935_noise_floor_level);
}