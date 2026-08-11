################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/examples/widgets/slider/lv_example_slider_1.c \
../grapics/examples/widgets/slider/lv_example_slider_2.c \
../grapics/examples/widgets/slider/lv_example_slider_3.c 

OBJS += \
./grapics/examples/widgets/slider/lv_example_slider_1.o \
./grapics/examples/widgets/slider/lv_example_slider_2.o \
./grapics/examples/widgets/slider/lv_example_slider_3.o 

C_DEPS += \
./grapics/examples/widgets/slider/lv_example_slider_1.d \
./grapics/examples/widgets/slider/lv_example_slider_2.d \
./grapics/examples/widgets/slider/lv_example_slider_3.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/examples/widgets/slider/%.o grapics/examples/widgets/slider/%.su grapics/examples/widgets/slider/%.cyclo: ../grapics/examples/widgets/slider/%.c grapics/examples/widgets/slider/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-examples-2f-widgets-2f-slider

clean-grapics-2f-examples-2f-widgets-2f-slider:
	-$(RM) ./grapics/examples/widgets/slider/lv_example_slider_1.cyclo ./grapics/examples/widgets/slider/lv_example_slider_1.d ./grapics/examples/widgets/slider/lv_example_slider_1.o ./grapics/examples/widgets/slider/lv_example_slider_1.su ./grapics/examples/widgets/slider/lv_example_slider_2.cyclo ./grapics/examples/widgets/slider/lv_example_slider_2.d ./grapics/examples/widgets/slider/lv_example_slider_2.o ./grapics/examples/widgets/slider/lv_example_slider_2.su ./grapics/examples/widgets/slider/lv_example_slider_3.cyclo ./grapics/examples/widgets/slider/lv_example_slider_3.d ./grapics/examples/widgets/slider/lv_example_slider_3.o ./grapics/examples/widgets/slider/lv_example_slider_3.su

.PHONY: clean-grapics-2f-examples-2f-widgets-2f-slider

