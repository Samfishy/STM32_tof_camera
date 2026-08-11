################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/examples/event/lv_example_event_1.c \
../grapics/examples/event/lv_example_event_2.c \
../grapics/examples/event/lv_example_event_3.c \
../grapics/examples/event/lv_example_event_4.c 

OBJS += \
./grapics/examples/event/lv_example_event_1.o \
./grapics/examples/event/lv_example_event_2.o \
./grapics/examples/event/lv_example_event_3.o \
./grapics/examples/event/lv_example_event_4.o 

C_DEPS += \
./grapics/examples/event/lv_example_event_1.d \
./grapics/examples/event/lv_example_event_2.d \
./grapics/examples/event/lv_example_event_3.d \
./grapics/examples/event/lv_example_event_4.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/examples/event/%.o grapics/examples/event/%.su grapics/examples/event/%.cyclo: ../grapics/examples/event/%.c grapics/examples/event/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-examples-2f-event

clean-grapics-2f-examples-2f-event:
	-$(RM) ./grapics/examples/event/lv_example_event_1.cyclo ./grapics/examples/event/lv_example_event_1.d ./grapics/examples/event/lv_example_event_1.o ./grapics/examples/event/lv_example_event_1.su ./grapics/examples/event/lv_example_event_2.cyclo ./grapics/examples/event/lv_example_event_2.d ./grapics/examples/event/lv_example_event_2.o ./grapics/examples/event/lv_example_event_2.su ./grapics/examples/event/lv_example_event_3.cyclo ./grapics/examples/event/lv_example_event_3.d ./grapics/examples/event/lv_example_event_3.o ./grapics/examples/event/lv_example_event_3.su ./grapics/examples/event/lv_example_event_4.cyclo ./grapics/examples/event/lv_example_event_4.d ./grapics/examples/event/lv_example_event_4.o ./grapics/examples/event/lv_example_event_4.su

.PHONY: clean-grapics-2f-examples-2f-event

