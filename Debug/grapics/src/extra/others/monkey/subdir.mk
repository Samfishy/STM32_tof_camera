################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/src/extra/others/monkey/lv_monkey.c 

OBJS += \
./grapics/src/extra/others/monkey/lv_monkey.o 

C_DEPS += \
./grapics/src/extra/others/monkey/lv_monkey.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/src/extra/others/monkey/%.o grapics/src/extra/others/monkey/%.su grapics/src/extra/others/monkey/%.cyclo: ../grapics/src/extra/others/monkey/%.c grapics/src/extra/others/monkey/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-src-2f-extra-2f-others-2f-monkey

clean-grapics-2f-src-2f-extra-2f-others-2f-monkey:
	-$(RM) ./grapics/src/extra/others/monkey/lv_monkey.cyclo ./grapics/src/extra/others/monkey/lv_monkey.d ./grapics/src/extra/others/monkey/lv_monkey.o ./grapics/src/extra/others/monkey/lv_monkey.su

.PHONY: clean-grapics-2f-src-2f-extra-2f-others-2f-monkey

