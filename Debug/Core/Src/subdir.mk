################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/display_tft.c \
../Core/Src/flash.c \
../Core/Src/fonts.c \
../Core/Src/interpolation_heatmap.c \
../Core/Src/lvgl_func.c \
../Core/Src/main.c \
../Core/Src/platform.c \
../Core/Src/st7735.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tof.c \
../Core/Src/vl53l5cx_api.c 

OBJS += \
./Core/Src/display_tft.o \
./Core/Src/flash.o \
./Core/Src/fonts.o \
./Core/Src/interpolation_heatmap.o \
./Core/Src/lvgl_func.o \
./Core/Src/main.o \
./Core/Src/platform.o \
./Core/Src/st7735.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tof.o \
./Core/Src/vl53l5cx_api.o 

C_DEPS += \
./Core/Src/display_tft.d \
./Core/Src/flash.d \
./Core/Src/fonts.d \
./Core/Src/interpolation_heatmap.d \
./Core/Src/lvgl_func.d \
./Core/Src/main.d \
./Core/Src/platform.d \
./Core/Src/st7735.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tof.d \
./Core/Src/vl53l5cx_api.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/display_tft.cyclo ./Core/Src/display_tft.d ./Core/Src/display_tft.o ./Core/Src/display_tft.su ./Core/Src/flash.cyclo ./Core/Src/flash.d ./Core/Src/flash.o ./Core/Src/flash.su ./Core/Src/fonts.cyclo ./Core/Src/fonts.d ./Core/Src/fonts.o ./Core/Src/fonts.su ./Core/Src/interpolation_heatmap.cyclo ./Core/Src/interpolation_heatmap.d ./Core/Src/interpolation_heatmap.o ./Core/Src/interpolation_heatmap.su ./Core/Src/lvgl_func.cyclo ./Core/Src/lvgl_func.d ./Core/Src/lvgl_func.o ./Core/Src/lvgl_func.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/platform.cyclo ./Core/Src/platform.d ./Core/Src/platform.o ./Core/Src/platform.su ./Core/Src/st7735.cyclo ./Core/Src/st7735.d ./Core/Src/st7735.o ./Core/Src/st7735.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_hal_timebase_tim.cyclo ./Core/Src/stm32f4xx_hal_timebase_tim.d ./Core/Src/stm32f4xx_hal_timebase_tim.o ./Core/Src/stm32f4xx_hal_timebase_tim.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tof.cyclo ./Core/Src/tof.d ./Core/Src/tof.o ./Core/Src/tof.su ./Core/Src/vl53l5cx_api.cyclo ./Core/Src/vl53l5cx_api.d ./Core/Src/vl53l5cx_api.o ./Core/Src/vl53l5cx_api.su

.PHONY: clean-Core-2f-Src

