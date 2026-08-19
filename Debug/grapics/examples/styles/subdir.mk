################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/examples/styles/lv_example_style_1.c \
../grapics/examples/styles/lv_example_style_10.c \
../grapics/examples/styles/lv_example_style_11.c \
../grapics/examples/styles/lv_example_style_12.c \
../grapics/examples/styles/lv_example_style_13.c \
../grapics/examples/styles/lv_example_style_14.c \
../grapics/examples/styles/lv_example_style_15.c \
../grapics/examples/styles/lv_example_style_2.c \
../grapics/examples/styles/lv_example_style_3.c \
../grapics/examples/styles/lv_example_style_4.c \
../grapics/examples/styles/lv_example_style_5.c \
../grapics/examples/styles/lv_example_style_6.c \
../grapics/examples/styles/lv_example_style_7.c \
../grapics/examples/styles/lv_example_style_8.c \
../grapics/examples/styles/lv_example_style_9.c 

OBJS += \
./grapics/examples/styles/lv_example_style_1.o \
./grapics/examples/styles/lv_example_style_10.o \
./grapics/examples/styles/lv_example_style_11.o \
./grapics/examples/styles/lv_example_style_12.o \
./grapics/examples/styles/lv_example_style_13.o \
./grapics/examples/styles/lv_example_style_14.o \
./grapics/examples/styles/lv_example_style_15.o \
./grapics/examples/styles/lv_example_style_2.o \
./grapics/examples/styles/lv_example_style_3.o \
./grapics/examples/styles/lv_example_style_4.o \
./grapics/examples/styles/lv_example_style_5.o \
./grapics/examples/styles/lv_example_style_6.o \
./grapics/examples/styles/lv_example_style_7.o \
./grapics/examples/styles/lv_example_style_8.o \
./grapics/examples/styles/lv_example_style_9.o 

C_DEPS += \
./grapics/examples/styles/lv_example_style_1.d \
./grapics/examples/styles/lv_example_style_10.d \
./grapics/examples/styles/lv_example_style_11.d \
./grapics/examples/styles/lv_example_style_12.d \
./grapics/examples/styles/lv_example_style_13.d \
./grapics/examples/styles/lv_example_style_14.d \
./grapics/examples/styles/lv_example_style_15.d \
./grapics/examples/styles/lv_example_style_2.d \
./grapics/examples/styles/lv_example_style_3.d \
./grapics/examples/styles/lv_example_style_4.d \
./grapics/examples/styles/lv_example_style_5.d \
./grapics/examples/styles/lv_example_style_6.d \
./grapics/examples/styles/lv_example_style_7.d \
./grapics/examples/styles/lv_example_style_8.d \
./grapics/examples/styles/lv_example_style_9.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/examples/styles/%.o grapics/examples/styles/%.su grapics/examples/styles/%.cyclo: ../grapics/examples/styles/%.c grapics/examples/styles/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-examples-2f-styles

clean-grapics-2f-examples-2f-styles:
	-$(RM) ./grapics/examples/styles/lv_example_style_1.cyclo ./grapics/examples/styles/lv_example_style_1.d ./grapics/examples/styles/lv_example_style_1.o ./grapics/examples/styles/lv_example_style_1.su ./grapics/examples/styles/lv_example_style_10.cyclo ./grapics/examples/styles/lv_example_style_10.d ./grapics/examples/styles/lv_example_style_10.o ./grapics/examples/styles/lv_example_style_10.su ./grapics/examples/styles/lv_example_style_11.cyclo ./grapics/examples/styles/lv_example_style_11.d ./grapics/examples/styles/lv_example_style_11.o ./grapics/examples/styles/lv_example_style_11.su ./grapics/examples/styles/lv_example_style_12.cyclo ./grapics/examples/styles/lv_example_style_12.d ./grapics/examples/styles/lv_example_style_12.o ./grapics/examples/styles/lv_example_style_12.su ./grapics/examples/styles/lv_example_style_13.cyclo ./grapics/examples/styles/lv_example_style_13.d ./grapics/examples/styles/lv_example_style_13.o ./grapics/examples/styles/lv_example_style_13.su ./grapics/examples/styles/lv_example_style_14.cyclo ./grapics/examples/styles/lv_example_style_14.d ./grapics/examples/styles/lv_example_style_14.o ./grapics/examples/styles/lv_example_style_14.su ./grapics/examples/styles/lv_example_style_15.cyclo ./grapics/examples/styles/lv_example_style_15.d ./grapics/examples/styles/lv_example_style_15.o ./grapics/examples/styles/lv_example_style_15.su ./grapics/examples/styles/lv_example_style_2.cyclo ./grapics/examples/styles/lv_example_style_2.d ./grapics/examples/styles/lv_example_style_2.o ./grapics/examples/styles/lv_example_style_2.su ./grapics/examples/styles/lv_example_style_3.cyclo ./grapics/examples/styles/lv_example_style_3.d ./grapics/examples/styles/lv_example_style_3.o ./grapics/examples/styles/lv_example_style_3.su ./grapics/examples/styles/lv_example_style_4.cyclo ./grapics/examples/styles/lv_example_style_4.d ./grapics/examples/styles/lv_example_style_4.o ./grapics/examples/styles/lv_example_style_4.su ./grapics/examples/styles/lv_example_style_5.cyclo ./grapics/examples/styles/lv_example_style_5.d ./grapics/examples/styles/lv_example_style_5.o ./grapics/examples/styles/lv_example_style_5.su ./grapics/examples/styles/lv_example_style_6.cyclo ./grapics/examples/styles/lv_example_style_6.d ./grapics/examples/styles/lv_example_style_6.o ./grapics/examples/styles/lv_example_style_6.su ./grapics/examples/styles/lv_example_style_7.cyclo ./grapics/examples/styles/lv_example_style_7.d ./grapics/examples/styles/lv_example_style_7.o ./grapics/examples/styles/lv_example_style_7.su ./grapics/examples/styles/lv_example_style_8.cyclo ./grapics/examples/styles/lv_example_style_8.d ./grapics/examples/styles/lv_example_style_8.o ./grapics/examples/styles/lv_example_style_8.su ./grapics/examples/styles/lv_example_style_9.cyclo ./grapics/examples/styles/lv_example_style_9.d ./grapics/examples/styles/lv_example_style_9.o ./grapics/examples/styles/lv_example_style_9.su

.PHONY: clean-grapics-2f-examples-2f-styles

