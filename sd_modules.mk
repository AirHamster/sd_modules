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
SD_INC += ./sd_modules/mpu9250
endif

#UBLOX NEO-M8 GPS modules series
ifeq ($(USE_UBLOX_GPS_MODULE), TRUE)
SD_SRC += ./sd_modules/neo-m8/neo-m8.c
SD_SRC += ./sd_modules/neo-m8/minmea.c
SD_INC += ./sd_modules/neo-m8
endif

#DIGI XBee modules
ifeq ($(USE_XBEE_868_MODULE), TRUE)
SD_SRC += ./sd_modules/xbee/xbee.c
SD_INC += ./sd_modules/xbee
endif

# Shared variables
ALLCSRC += $(SD_SRC)
ALLINC  += $(SD_INC)