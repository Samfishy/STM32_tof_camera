################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/src/extra/libs/fsdrv/lv_fs_fatfs.c \
../grapics/src/extra/libs/fsdrv/lv_fs_littlefs.c \
../grapics/src/extra/libs/fsdrv/lv_fs_posix.c \
../grapics/src/extra/libs/fsdrv/lv_fs_stdio.c \
../grapics/src/extra/libs/fsdrv/lv_fs_win32.c 

OBJS += \
./grapics/src/extra/libs/fsdrv/lv_fs_fatfs.o \
./grapics/src/extra/libs/fsdrv/lv_fs_littlefs.o \
./grapics/src/extra/libs/fsdrv/lv_fs_posix.o \
./grapics/src/extra/libs/fsdrv/lv_fs_stdio.o \
./grapics/src/extra/libs/fsdrv/lv_fs_win32.o 

C_DEPS += \
./grapics/src/extra/libs/fsdrv/lv_fs_fatfs.d \
./grapics/src/extra/libs/fsdrv/lv_fs_littlefs.d \
./grapics/src/extra/libs/fsdrv/lv_fs_posix.d \
./grapics/src/extra/libs/fsdrv/lv_fs_stdio.d \
./grapics/src/extra/libs/fsdrv/lv_fs_win32.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/src/extra/libs/fsdrv/%.o grapics/src/extra/libs/fsdrv/%.su grapics/src/extra/libs/fsdrv/%.cyclo: ../grapics/src/extra/libs/fsdrv/%.c grapics/src/extra/libs/fsdrv/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-src-2f-extra-2f-libs-2f-fsdrv

clean-grapics-2f-src-2f-extra-2f-libs-2f-fsdrv:
	-$(RM) ./grapics/src/extra/libs/fsdrv/lv_fs_fatfs.cyclo ./grapics/src/extra/libs/fsdrv/lv_fs_fatfs.d ./grapics/src/extra/libs/fsdrv/lv_fs_fatfs.o ./grapics/src/extra/libs/fsdrv/lv_fs_fatfs.su ./grapics/src/extra/libs/fsdrv/lv_fs_littlefs.cyclo ./grapics/src/extra/libs/fsdrv/lv_fs_littlefs.d ./grapics/src/extra/libs/fsdrv/lv_fs_littlefs.o ./grapics/src/extra/libs/fsdrv/lv_fs_littlefs.su ./grapics/src/extra/libs/fsdrv/lv_fs_posix.cyclo ./grapics/src/extra/libs/fsdrv/lv_fs_posix.d ./grapics/src/extra/libs/fsdrv/lv_fs_posix.o ./grapics/src/extra/libs/fsdrv/lv_fs_posix.su ./grapics/src/extra/libs/fsdrv/lv_fs_stdio.cyclo ./grapics/src/extra/libs/fsdrv/lv_fs_stdio.d ./grapics/src/extra/libs/fsdrv/lv_fs_stdio.o ./grapics/src/extra/libs/fsdrv/lv_fs_stdio.su ./grapics/src/extra/libs/fsdrv/lv_fs_win32.cyclo ./grapics/src/extra/libs/fsdrv/lv_fs_win32.d ./grapics/src/extra/libs/fsdrv/lv_fs_win32.o ./grapics/src/extra/libs/fsdrv/lv_fs_win32.su

.PHONY: clean-grapics-2f-src-2f-extra-2f-libs-2f-fsdrv

