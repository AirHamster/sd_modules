# List of all the board related files.
#Debug shell - often used on USART1 to debug firmware
ifeq ($(USE_DEBUG_SHELL), TRUE)
# Compiler options here.
SD_SRC += ./sd_modules/dbg_shell_cmd/dbg_shell_cmd.c

# Required include directories
SD_INC += ./sd_modules/dbg_shell_cmd
endif

#SailData shell - often used on USART1 to communicate with PC
ifeq ($(USE_SD_SHELL), TRUE)
# Compiler options here.
SD_SRC += ./sd_modules/sd_shell_cmd/sd_shell_cmd.c

# Required include directories
SD_INC += ./sd_modules/sd_shell_cmd
endif

# Shared variables
ALLCSRC += $(SD_SRC)
ALLINC  += $(SD_INC)