################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/examples/widgets/arc/lv_example_arc_1.c \
../grapics/examples/widgets/arc/lv_example_arc_2.c 

OBJS += \
./grapics/examples/widgets/arc/lv_example_arc_1.o \
./grapics/examples/widgets/arc/lv_example_arc_2.o 

C_DEPS += \
./grapics/examples/widgets/arc/lv_example_arc_1.d \
./grapics/examples/widgets/arc/lv_example_arc_2.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/examples/widgets/arc/%.o grapics/examples/widgets/arc/%.su grapics/examples/widgets/arc/%.cyclo: ../grapics/examples/widgets/arc/%.c grapics/examples/widgets/arc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-examples-2f-widgets-2f-arc

clean-grapics-2f-examples-2f-widgets-2f-arc:
	-$(RM) ./grapics/examples/widgets/arc/lv_example_arc_1.cyclo ./grapics/examples/widgets/arc/lv_example_arc_1.d ./grapics/examples/widgets/arc/lv_example_arc_1.o ./grapics/examples/widgets/arc/lv_example_arc_1.su ./grapics/examples/widgets/arc/lv_example_arc_2.cyclo ./grapics/examples/widgets/arc/lv_example_arc_2.d ./grapics/examples/widgets/arc/lv_example_arc_2.o ./grapics/examples/widgets/arc/lv_example_arc_2.su

.PHONY: clean-grapics-2f-examples-2f-widgets-2f-arc

