################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/examples/widgets/dropdown/lv_example_dropdown_1.c \
../grapics/examples/widgets/dropdown/lv_example_dropdown_2.c \
../grapics/examples/widgets/dropdown/lv_example_dropdown_3.c 

OBJS += \
./grapics/examples/widgets/dropdown/lv_example_dropdown_1.o \
./grapics/examples/widgets/dropdown/lv_example_dropdown_2.o \
./grapics/examples/widgets/dropdown/lv_example_dropdown_3.o 

C_DEPS += \
./grapics/examples/widgets/dropdown/lv_example_dropdown_1.d \
./grapics/examples/widgets/dropdown/lv_example_dropdown_2.d \
./grapics/examples/widgets/dropdown/lv_example_dropdown_3.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/examples/widgets/dropdown/%.o grapics/examples/widgets/dropdown/%.su grapics/examples/widgets/dropdown/%.cyclo: ../grapics/examples/widgets/dropdown/%.c grapics/examples/widgets/dropdown/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-examples-2f-widgets-2f-dropdown

clean-grapics-2f-examples-2f-widgets-2f-dropdown:
	-$(RM) ./grapics/examples/widgets/dropdown/lv_example_dropdown_1.cyclo ./grapics/examples/widgets/dropdown/lv_example_dropdown_1.d ./grapics/examples/widgets/dropdown/lv_example_dropdown_1.o ./grapics/examples/widgets/dropdown/lv_example_dropdown_1.su ./grapics/examples/widgets/dropdown/lv_example_dropdown_2.cyclo ./grapics/examples/widgets/dropdown/lv_example_dropdown_2.d ./grapics/examples/widgets/dropdown/lv_example_dropdown_2.o ./grapics/examples/widgets/dropdown/lv_example_dropdown_2.su ./grapics/examples/widgets/dropdown/lv_example_dropdown_3.cyclo ./grapics/examples/widgets/dropdown/lv_example_dropdown_3.d ./grapics/examples/widgets/dropdown/lv_example_dropdown_3.o ./grapics/examples/widgets/dropdown/lv_example_dropdown_3.su

.PHONY: clean-grapics-2f-examples-2f-widgets-2f-dropdown

