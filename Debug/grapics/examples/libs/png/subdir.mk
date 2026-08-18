################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/examples/libs/png/img_wink_png.c \
../grapics/examples/libs/png/lv_example_png_1.c 

OBJS += \
./grapics/examples/libs/png/img_wink_png.o \
./grapics/examples/libs/png/lv_example_png_1.o 

C_DEPS += \
./grapics/examples/libs/png/img_wink_png.d \
./grapics/examples/libs/png/lv_example_png_1.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/examples/libs/png/%.o grapics/examples/libs/png/%.su grapics/examples/libs/png/%.cyclo: ../grapics/examples/libs/png/%.c grapics/examples/libs/png/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-examples-2f-libs-2f-png

clean-grapics-2f-examples-2f-libs-2f-png:
	-$(RM) ./grapics/examples/libs/png/img_wink_png.cyclo ./grapics/examples/libs/png/img_wink_png.d ./grapics/examples/libs/png/img_wink_png.o ./grapics/examples/libs/png/img_wink_png.su ./grapics/examples/libs/png/lv_example_png_1.cyclo ./grapics/examples/libs/png/lv_example_png_1.d ./grapics/examples/libs/png/lv_example_png_1.o ./grapics/examples/libs/png/lv_example_png_1.su

.PHONY: clean-grapics-2f-examples-2f-libs-2f-png

