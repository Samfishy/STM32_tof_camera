################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/examples/others/ime/lv_example_ime_pinyin_1.c \
../grapics/examples/others/ime/lv_example_ime_pinyin_2.c 

OBJS += \
./grapics/examples/others/ime/lv_example_ime_pinyin_1.o \
./grapics/examples/others/ime/lv_example_ime_pinyin_2.o 

C_DEPS += \
./grapics/examples/others/ime/lv_example_ime_pinyin_1.d \
./grapics/examples/others/ime/lv_example_ime_pinyin_2.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/examples/others/ime/%.o grapics/examples/others/ime/%.su grapics/examples/others/ime/%.cyclo: ../grapics/examples/others/ime/%.c grapics/examples/others/ime/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-examples-2f-others-2f-ime

clean-grapics-2f-examples-2f-others-2f-ime:
	-$(RM) ./grapics/examples/others/ime/lv_example_ime_pinyin_1.cyclo ./grapics/examples/others/ime/lv_example_ime_pinyin_1.d ./grapics/examples/others/ime/lv_example_ime_pinyin_1.o ./grapics/examples/others/ime/lv_example_ime_pinyin_1.su ./grapics/examples/others/ime/lv_example_ime_pinyin_2.cyclo ./grapics/examples/others/ime/lv_example_ime_pinyin_2.d ./grapics/examples/others/ime/lv_example_ime_pinyin_2.o ./grapics/examples/others/ime/lv_example_ime_pinyin_2.su

.PHONY: clean-grapics-2f-examples-2f-others-2f-ime

