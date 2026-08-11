################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/examples/widgets/calendar/lv_example_calendar_1.c 

OBJS += \
./grapics/examples/widgets/calendar/lv_example_calendar_1.o 

C_DEPS += \
./grapics/examples/widgets/calendar/lv_example_calendar_1.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/examples/widgets/calendar/%.o grapics/examples/widgets/calendar/%.su grapics/examples/widgets/calendar/%.cyclo: ../grapics/examples/widgets/calendar/%.c grapics/examples/widgets/calendar/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-examples-2f-widgets-2f-calendar

clean-grapics-2f-examples-2f-widgets-2f-calendar:
	-$(RM) ./grapics/examples/widgets/calendar/lv_example_calendar_1.cyclo ./grapics/examples/widgets/calendar/lv_example_calendar_1.d ./grapics/examples/widgets/calendar/lv_example_calendar_1.o ./grapics/examples/widgets/calendar/lv_example_calendar_1.su

.PHONY: clean-grapics-2f-examples-2f-widgets-2f-calendar

