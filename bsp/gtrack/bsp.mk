include bsp/$(TRG)/board.mk
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/startup_stm32l1xx.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32L1xx/platform.mk
LDSCRIPT= $(STARTUPLD)/STM32L152xB.ld

MCU  = cortex-m3

SRC:= drivers/i2c_api.c \
      drivers/motion/accelGyro.c \
      drivers/network/gsm_api.c \
      drivers/network/gsm_ll.c \
      drivers/utils.c \

CSRC+=$(patsubst %.c, %.o, $(SRC))
