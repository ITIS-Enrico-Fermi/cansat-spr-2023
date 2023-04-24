# Flight

Handle flight operations during the launch.

This application reads data from the sensors, gather it all together and sends
a packet down the LoRa link to the ground station.


## Implemented sensors

These sensors are curretly implemented:

| Name     | Device path       | Provides                                       |
|----------|-------------------|------------------------------------------------|
| VEML6070 | /dev/sensor/uv0   | UV light measurement                           |
| MPU6050  | /dev/sensor/gyro0 | Acceleration, rotational momentum, temperature |
| BMP280   | /dev/press0       | Pressure                                       |

*BMP280 kernel space code must be checked, and its device path changed to `/dev/sensor/press0`.


## Involved code

Different areas of both the kernel and the userspace code are involved to make this application
work, for mnemonics a list of the source files directly or indirectly used is provided:

- `nuttx/boards/arm/cxd56xx/spresense/src/cxd56_bringup.c`, the initialisation code where sensors
are instantiated and prepared. This is more accurately described in [the top level README](../../README.md).
Device paths are specified here too;

- `nuttx/boards/arm/cxd56xx/common/src/cxd56_sensors.c` configures the BMP280, among other standard
sensors on Spresense boards.

- `nuttx/drivers/sensors/mpu60x0.c` a character driver for the MPU6050. The data "protocol" is both
described here and on the datasheet;

- `nuttx/drivers/sensors/veml6070.c` a simple character driver that expects reads only, of exactly
2 bytes each time. These correspond to the dimension of data outputted by the sensor: a 16 bit unsigned
integer representing the intensity of UV light.

## Related Kconfig

These Kconfig settings are required in order for this application to work:

- Device Drivers
    - I2C Driver Support (with I2C Character Driver)
    - Sensor Device Support
        - Bosch BMP280 Barometric Pressure Sensor
            - 400000 I2C Frequency
        - Invensense MPU60x0
            - I2C Interface
            - 400000 I2C Frequency
        - Vishay VEML6070 UV-A Light Sensor
            - 400000 I2C Frequency

## I2C

BMP280, VEML6070 and MPU6050 all share a single I2C bus (tied to the controller 0 as defined in
`cxd56_bringup.c`).

These devices have an I2C address that identifies them, no conflicts should arise as the three addresses
are different:

- VEML6070: `0x38`

- BMP280: `0x76`

- MPU6050: `0x68`
