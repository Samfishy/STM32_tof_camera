################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/examples/widgets/chart/lv_example_chart_1.c \
../grapics/examples/widgets/chart/lv_example_chart_2.c \
../grapics/examples/widgets/chart/lv_example_chart_3.c \
../grapics/examples/widgets/chart/lv_example_chart_4.c \
../grapics/examples/widgets/chart/lv_example_chart_5.c \
../grapics/examples/widgets/chart/lv_example_chart_6.c \
../grapics/examples/widgets/chart/lv_example_chart_7.c \
../grapics/examples/widgets/chart/lv_example_chart_8.c \
../grapics/examples/widgets/chart/lv_example_chart_9.c 

OBJS += \
./grapics/examples/widgets/chart/lv_example_chart_1.o \
./grapics/examples/widgets/chart/lv_example_chart_2.o \
./grapics/examples/widgets/chart/lv_example_chart_3.o \
./grapics/examples/widgets/chart/lv_example_chart_4.o \
./grapics/examples/widgets/chart/lv_example_chart_5.o \
./grapics/examples/widgets/chart/lv_example_chart_6.o \
./grapics/examples/widgets/chart/lv_example_chart_7.o \
./grapics/examples/widgets/chart/lv_example_chart_8.o \
./grapics/examples/widgets/chart/lv_example_chart_9.o 

C_DEPS += \
./grapics/examples/widgets/chart/lv_example_chart_1.d \
./grapics/examples/widgets/chart/lv_example_chart_2.d \
./grapics/examples/widgets/chart/lv_example_chart_3.d \
./grapics/examples/widgets/chart/lv_example_chart_4.d \
./grapics/examples/widgets/chart/lv_example_chart_5.d \
./grapics/examples/widgets/chart/lv_example_chart_6.d \
./grapics/examples/widgets/chart/lv_example_chart_7.d \
./grapics/examples/widgets/chart/lv_example_chart_8.d \
./grapics/examples/widgets/chart/lv_example_chart_9.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/examples/widgets/chart/%.o grapics/examples/widgets/chart/%.su grapics/examples/widgets/chart/%.cyclo: ../grapics/examples/widgets/chart/%.c grapics/examples/widgets/chart/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-examples-2f-widgets-2f-chart

clean-grapics-2f-examples-2f-widgets-2f-chart:
	-$(RM) ./grapics/examples/widgets/chart/lv_example_chart_1.cyclo ./grapics/examples/widgets/chart/lv_example_chart_1.d ./grapics/examples/widgets/chart/lv_example_chart_1.o ./grapics/examples/widgets/chart/lv_example_chart_1.su ./grapics/examples/widgets/chart/lv_example_chart_2.cyclo ./grapics/examples/widgets/chart/lv_example_chart_2.d ./grapics/examples/widgets/chart/lv_example_chart_2.o ./grapics/examples/widgets/chart/lv_example_chart_2.su ./grapics/examples/widgets/chart/lv_example_chart_3.cyclo ./grapics/examples/widgets/chart/lv_example_chart_3.d ./grapics/examples/widgets/chart/lv_example_chart_3.o ./grapics/examples/widgets/chart/lv_example_chart_3.su ./grapics/examples/widgets/chart/lv_example_chart_4.cyclo ./grapics/examples/widgets/chart/lv_example_chart_4.d ./grapics/examples/widgets/chart/lv_example_chart_4.o ./grapics/examples/widgets/chart/lv_example_chart_4.su ./grapics/examples/widgets/chart/lv_example_chart_5.cyclo ./grapics/examples/widgets/chart/lv_example_chart_5.d ./grapics/examples/widgets/chart/lv_example_chart_5.o ./grapics/examples/widgets/chart/lv_example_chart_5.su ./grapics/examples/widgets/chart/lv_example_chart_6.cyclo ./grapics/examples/widgets/chart/lv_example_chart_6.d ./grapics/examples/widgets/chart/lv_example_chart_6.o ./grapics/examples/widgets/chart/lv_example_chart_6.su ./grapics/examples/widgets/chart/lv_example_chart_7.cyclo ./grapics/examples/widgets/chart/lv_example_chart_7.d ./grapics/examples/widgets/chart/lv_example_chart_7.o ./grapics/examples/widgets/chart/lv_example_chart_7.su ./grapics/examples/widgets/chart/lv_example_chart_8.cyclo ./grapics/examples/widgets/chart/lv_example_chart_8.d ./grapics/examples/widgets/chart/lv_example_chart_8.o ./grapics/examples/widgets/chart/lv_example_chart_8.su ./grapics/examples/widgets/chart/lv_example_chart_9.cyclo ./grapics/examples/widgets/chart/lv_example_chart_9.d ./grapics/examples/widgets/chart/lv_example_chart_9.o ./grapics/examples/widgets/chart/lv_example_chart_9.su

.PHONY: clean-grapics-2f-examples-2f-widgets-2f-chart

