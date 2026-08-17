################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/examples/layouts/flex/lv_example_flex_1.c \
../grapics/examples/layouts/flex/lv_example_flex_2.c \
../grapics/examples/layouts/flex/lv_example_flex_3.c \
../grapics/examples/layouts/flex/lv_example_flex_4.c \
../grapics/examples/layouts/flex/lv_example_flex_5.c \
../grapics/examples/layouts/flex/lv_example_flex_6.c 

OBJS += \
./grapics/examples/layouts/flex/lv_example_flex_1.o \
./grapics/examples/layouts/flex/lv_example_flex_2.o \
./grapics/examples/layouts/flex/lv_example_flex_3.o \
./grapics/examples/layouts/flex/lv_example_flex_4.o \
./grapics/examples/layouts/flex/lv_example_flex_5.o \
./grapics/examples/layouts/flex/lv_example_flex_6.o 

C_DEPS += \
./grapics/examples/layouts/flex/lv_example_flex_1.d \
./grapics/examples/layouts/flex/lv_example_flex_2.d \
./grapics/examples/layouts/flex/lv_example_flex_3.d \
./grapics/examples/layouts/flex/lv_example_flex_4.d \
./grapics/examples/layouts/flex/lv_example_flex_5.d \
./grapics/examples/layouts/flex/lv_example_flex_6.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/examples/layouts/flex/%.o grapics/examples/layouts/flex/%.su grapics/examples/layouts/flex/%.cyclo: ../grapics/examples/layouts/flex/%.c grapics/examples/layouts/flex/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-examples-2f-layouts-2f-flex

clean-grapics-2f-examples-2f-layouts-2f-flex:
	-$(RM) ./grapics/examples/layouts/flex/lv_example_flex_1.cyclo ./grapics/examples/layouts/flex/lv_example_flex_1.d ./grapics/examples/layouts/flex/lv_example_flex_1.o ./grapics/examples/layouts/flex/lv_example_flex_1.su ./grapics/examples/layouts/flex/lv_example_flex_2.cyclo ./grapics/examples/layouts/flex/lv_example_flex_2.d ./grapics/examples/layouts/flex/lv_example_flex_2.o ./grapics/examples/layouts/flex/lv_example_flex_2.su ./grapics/examples/layouts/flex/lv_example_flex_3.cyclo ./grapics/examples/layouts/flex/lv_example_flex_3.d ./grapics/examples/layouts/flex/lv_example_flex_3.o ./grapics/examples/layouts/flex/lv_example_flex_3.su ./grapics/examples/layouts/flex/lv_example_flex_4.cyclo ./grapics/examples/layouts/flex/lv_example_flex_4.d ./grapics/examples/layouts/flex/lv_example_flex_4.o ./grapics/examples/layouts/flex/lv_example_flex_4.su ./grapics/examples/layouts/flex/lv_example_flex_5.cyclo ./grapics/examples/layouts/flex/lv_example_flex_5.d ./grapics/examples/layouts/flex/lv_example_flex_5.o ./grapics/examples/layouts/flex/lv_example_flex_5.su ./grapics/examples/layouts/flex/lv_example_flex_6.cyclo ./grapics/examples/layouts/flex/lv_example_flex_6.d ./grapics/examples/layouts/flex/lv_example_flex_6.o ./grapics/examples/layouts/flex/lv_example_flex_6.su

.PHONY: clean-grapics-2f-examples-2f-layouts-2f-flex

