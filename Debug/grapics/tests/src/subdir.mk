################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/tests/src/lv_test_indev.c \
../grapics/tests/src/lv_test_init.c 

OBJS += \
./grapics/tests/src/lv_test_indev.o \
./grapics/tests/src/lv_test_init.o 

C_DEPS += \
./grapics/tests/src/lv_test_indev.d \
./grapics/tests/src/lv_test_init.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/tests/src/%.o grapics/tests/src/%.su grapics/tests/src/%.cyclo: ../grapics/tests/src/%.c grapics/tests/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-tests-2f-src

clean-grapics-2f-tests-2f-src:
	-$(RM) ./grapics/tests/src/lv_test_indev.cyclo ./grapics/tests/src/lv_test_indev.d ./grapics/tests/src/lv_test_indev.o ./grapics/tests/src/lv_test_indev.su ./grapics/tests/src/lv_test_init.cyclo ./grapics/tests/src/lv_test_init.d ./grapics/tests/src/lv_test_init.o ./grapics/tests/src/lv_test_init.su

.PHONY: clean-grapics-2f-tests-2f-src

