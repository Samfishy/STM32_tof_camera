################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/ui/images.c \
../grapics/ui/screens.c \
../grapics/ui/styles.c \
../grapics/ui/ui.c \
../grapics/ui/ui_image_bitmap_heatmap.c 

OBJS += \
./grapics/ui/images.o \
./grapics/ui/screens.o \
./grapics/ui/styles.o \
./grapics/ui/ui.o \
./grapics/ui/ui_image_bitmap_heatmap.o 

C_DEPS += \
./grapics/ui/images.d \
./grapics/ui/screens.d \
./grapics/ui/styles.d \
./grapics/ui/ui.d \
./grapics/ui/ui_image_bitmap_heatmap.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/ui/%.o grapics/ui/%.su grapics/ui/%.cyclo: ../grapics/ui/%.c grapics/ui/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-ui

clean-grapics-2f-ui:
	-$(RM) ./grapics/ui/images.cyclo ./grapics/ui/images.d ./grapics/ui/images.o ./grapics/ui/images.su ./grapics/ui/screens.cyclo ./grapics/ui/screens.d ./grapics/ui/screens.o ./grapics/ui/screens.su ./grapics/ui/styles.cyclo ./grapics/ui/styles.d ./grapics/ui/styles.o ./grapics/ui/styles.su ./grapics/ui/ui.cyclo ./grapics/ui/ui.d ./grapics/ui/ui.o ./grapics/ui/ui.su ./grapics/ui/ui_image_bitmap_heatmap.cyclo ./grapics/ui/ui_image_bitmap_heatmap.d ./grapics/ui/ui_image_bitmap_heatmap.o ./grapics/ui/ui_image_bitmap_heatmap.su

.PHONY: clean-grapics-2f-ui

