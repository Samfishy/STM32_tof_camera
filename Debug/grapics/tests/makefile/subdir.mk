################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/tests/makefile/test.c 

OBJS += \
./grapics/tests/makefile/test.o 

C_DEPS += \
./grapics/tests/makefile/test.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/tests/makefile/%.o grapics/tests/makefile/%.su grapics/tests/makefile/%.cyclo: ../grapics/tests/makefile/%.c grapics/tests/makefile/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-tests-2f-makefile

clean-grapics-2f-tests-2f-makefile:
	-$(RM) ./grapics/tests/makefile/test.cyclo ./grapics/tests/makefile/test.d ./grapics/tests/makefile/test.o ./grapics/tests/makefile/test.su

.PHONY: clean-grapics-2f-tests-2f-makefile

