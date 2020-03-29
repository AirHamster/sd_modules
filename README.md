# SD_Modules

[![Build Status](https://travis-ci.com/AirHamster/sd_modules.svg?token=KGT3nWDyeGNHSLPsrsXU&branch=master)](https://travis-ci.com/AirHamster/sd_modules)

##### SD_Modules is a repository with code base for all sensors and devices used in SailData project.

##### Current modules:
- Bosch 9-axis sensor driver
- Bosch 9-axis sensor with build-in fusion algorith driver
- TI BQ2560x battery charge IC driver
- TI BQ27441 fuel gauge driver
- Generic EEPROM driver
- Log sensor driver
- Documentation from chips manufactures
- Math parameters calculating
- Multi MCU communication driver
- SPI MicroSD driver with FatFS support
- Ublox GNSS module driver
- Ublox BLE module driver
- L4 power management with deep sleep mode driver
- Shell commands and output driver
- Tenso sensor driver
- Windsensor with SD-12 interface driver
- XBP9X-900 Digi module driver

# HOWTO

Every module works as separete thread. After call start_*name*_module from main.c file modules initializes needed varables in memory and from EEPROM.

Modules includes by defines in config.h in main firmware projects.

Firmware repositories:

Main device:
[https://github.com/AirHamster/sd_fw_main_board]

BLE sensor device:
[https://github.com/AirHamster/sd_sensor_box_fw]
