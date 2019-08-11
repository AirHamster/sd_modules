# List of all the board related files.
#Debug shell - often used on USART1 to debug firmware
ifeq ($(USE_DEBUG_SHELL), TRUE)
SD_SRC += ./sd_modules/dbg_shell_cmd/dbg_shell_cmd.c
SD_INC += ./sd_modules/dbg_shell_cmd
endif

#SailData shell - often used on USART1 to communicate with PC
ifeq ($(USE_SD_SHELL), TRUE)
SD_SRC += ./sd_modules/sd_shell_cmd/sd_shell_cmd.c
SD_INC += ./sd_modules/sd_shell_cmd
endif

#MPU9250 - 9-axis accel/gyro/magn chip
ifeq ($(USE_MPU_9250_MODULE), TRUE)
SD_SRC += ./sd_modules/mpu9250/mpu9250.c
SD_SRC += ./sd_modules/mpu9250/MadgwickAHRS.c
//SD_SRC += ./sd_modules/mpu9250/quaternionFilters.c
SD_INC += ./sd_modules/mpu9250
endif

#UBLOX NEO-M8 GPS modules series
ifeq ($(USE_UBLOX_GPS_MODULE), TRUE)
SD_SRC += ./sd_modules/neo-m8/neo-m8.c
SD_SRC += ./sd_modules/neo-m8/minmea.c
SD_SRC += ./sd_modules/neo-m8/neo_ubx.c
SD_INC += ./sd_modules/neo-m8
endif

#DIGI XBee modules
ifeq ($(USE_XBEE_868_MODULE), TRUE)
SD_SRC += ./sd_modules/xbee/xbee.c
SD_INC += ./sd_modules/xbee
endif

#EEPROM
ifeq ($(USE_EEPROM_MODULE), TRUE)
SD_SRC += ./sd_modules/eeprom/eeprom.c
SD_INC += ./sd_modules/eeprom
endif

#BNO055
ifeq ($(USE_BNO055_MODULE), TRUE)
SD_SRC += ./sd_modules/bno055/bno055.c
SD_SRC += ./sd_modules/bno055/bno055_i2c.c
SD_INC += ./sd_modules/bno055
endif

#FATFS
ifeq ($(USE_FATFS_MODULE), TRUE)
SD_SRC += ./sd_modules/fatfs/diskio.c
SD_SRC += ./sd_modules/fatfs/ff.c
SD_SRC += ./sd_modules/fatfs/ffsystem.c
SD_SRC += ./sd_modules/fatfs/ffunicode.c
SD_SRC += ./sd_modules/fatfs/filesystem.c
SD_INC += ./sd_modules/fatfs
endif

#WIND sensor
ifeq ($(USE_WINDSENSOR_MODULE), TRUE)
SD_SRC += ./sd_modules/windsensor/windsensor.c
SD_INC += ./sd_modules/windsensor
endif

# Shared variables
ALLCSRC += $(SD_SRC)
ALLINC  += $(SD_INC)
