#include "as3935.h"
#include <unistd.h>

int main()
{
    // AS3935 support spi mode on mode 1.
    struct _as3935 as3935;
    uint8_t value = 0;

    // Initial as3935 on spi0 and CS0.
    as3935_init(&as3935, "/dev/spidev0.0");

    // Power on as3935.
    as3935_power(&as3935, AS3935_POWER_STATUS_ON);

    // Config environment is indoor.
    SET_SENSOR_INDOOR(&as3935);

    // Config noise level.
    as3935_set_noise_floor_level(&as3935, AS3935_NOISE_FLOOR_LEVEL_1570_112);

    // Config minimum lightning.
    as3935_set_min_num_lightning(&as3935, AS3935_MIN_NUMBER_OF_LIGHTNING_NINE);

    
    while (1) {
        // Get event.
        as3935_get_status(&as3935, &value);
        switch (value) {
        case AS3935_INT_EVENT_NOISE_LEVEL_TOO_HIGH:
            printf("Noise level too high\n");
            break;
        case AS3935_INT_EVENT_DISTURBER_DETECTED:
            printf("Disturber detected\n");
            break;
        case AS3935_INT_EVENT_LIGHTNING_INTERRUPT:
            printf("Lightning detected\n");
            break;
        default:
            // No event.
            break;
        }

        // Get storm distance.
        as3935_get_storm_distance(&as3935, &value);
        if (value >= 0x3F) {
            printf("Storm out of range\n");
        } else if (value == 0x01 || value == 0) {
            printf("Storm is Overhead\n");
        } else {
            printf("Distance %d\n", value);
        }
        
        // Loop one second.
        sleep(1);
    }

    return 0;
}