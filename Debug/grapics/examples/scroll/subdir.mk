################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/examples/scroll/lv_example_scroll_1.c \
../grapics/examples/scroll/lv_example_scroll_2.c \
../grapics/examples/scroll/lv_example_scroll_3.c \
../grapics/examples/scroll/lv_example_scroll_4.c \
../grapics/examples/scroll/lv_example_scroll_5.c \
../grapics/examples/scroll/lv_example_scroll_6.c 

OBJS += \
./grapics/examples/scroll/lv_example_scroll_1.o \
./grapics/examples/scroll/lv_example_scroll_2.o \
./grapics/examples/scroll/lv_example_scroll_3.o \
./grapics/examples/scroll/lv_example_scroll_4.o \
./grapics/examples/scroll/lv_example_scroll_5.o \
./grapics/examples/scroll/lv_example_scroll_6.o 

C_DEPS += \
./grapics/examples/scroll/lv_example_scroll_1.d \
./grapics/examples/scroll/lv_example_scroll_2.d \
./grapics/examples/scroll/lv_example_scroll_3.d \
./grapics/examples/scroll/lv_example_scroll_4.d \
./grapics/examples/scroll/lv_example_scroll_5.d \
./grapics/examples/scroll/lv_example_scroll_6.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/examples/scroll/%.o grapics/examples/scroll/%.su grapics/examples/scroll/%.cyclo: ../grapics/examples/scroll/%.c grapics/examples/scroll/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-examples-2f-scroll

clean-grapics-2f-examples-2f-scroll:
	-$(RM) ./grapics/examples/scroll/lv_example_scroll_1.cyclo ./grapics/examples/scroll/lv_example_scroll_1.d ./grapics/examples/scroll/lv_example_scroll_1.o ./grapics/examples/scroll/lv_example_scroll_1.su ./grapics/examples/scroll/lv_example_scroll_2.cyclo ./grapics/examples/scroll/lv_example_scroll_2.d ./grapics/examples/scroll/lv_example_scroll_2.o ./grapics/examples/scroll/lv_example_scroll_2.su ./grapics/examples/scroll/lv_example_scroll_3.cyclo ./grapics/examples/scroll/lv_example_scroll_3.d ./grapics/examples/scroll/lv_example_scroll_3.o ./grapics/examples/scroll/lv_example_scroll_3.su ./grapics/examples/scroll/lv_example_scroll_4.cyclo ./grapics/examples/scroll/lv_example_scroll_4.d ./grapics/examples/scroll/lv_example_scroll_4.o ./grapics/examples/scroll/lv_example_scroll_4.su ./grapics/examples/scroll/lv_example_scroll_5.cyclo ./grapics/examples/scroll/lv_example_scroll_5.d ./grapics/examples/scroll/lv_example_scroll_5.o ./grapics/examples/scroll/lv_example_scroll_5.su ./grapics/examples/scroll/lv_example_scroll_6.cyclo ./grapics/examples/scroll/lv_example_scroll_6.d ./grapics/examples/scroll/lv_example_scroll_6.o ./grapics/examples/scroll/lv_example_scroll_6.su

.PHONY: clean-grapics-2f-examples-2f-scroll

