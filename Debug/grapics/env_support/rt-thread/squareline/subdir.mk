################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/env_support/rt-thread/squareline/lv_ui_entry.c 

OBJS += \
./grapics/env_support/rt-thread/squareline/lv_ui_entry.o 

C_DEPS += \
./grapics/env_support/rt-thread/squareline/lv_ui_entry.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/env_support/rt-thread/squareline/%.o grapics/env_support/rt-thread/squareline/%.su grapics/env_support/rt-thread/squareline/%.cyclo: ../grapics/env_support/rt-thread/squareline/%.c grapics/env_support/rt-thread/squareline/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-env_support-2f-rt-2d-thread-2f-squareline

clean-grapics-2f-env_support-2f-rt-2d-thread-2f-squareline:
	-$(RM) ./grapics/env_support/rt-thread/squareline/lv_ui_entry.cyclo ./grapics/env_support/rt-thread/squareline/lv_ui_entry.d ./grapics/env_support/rt-thread/squareline/lv_ui_entry.o ./grapics/env_support/rt-thread/squareline/lv_ui_entry.su

.PHONY: clean-grapics-2f-env_support-2f-rt-2d-thread-2f-squareline

