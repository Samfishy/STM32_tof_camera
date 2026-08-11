################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/tests/unity/unity.c \
../grapics/tests/unity/unity_support.c 

OBJS += \
./grapics/tests/unity/unity.o \
./grapics/tests/unity/unity_support.o 

C_DEPS += \
./grapics/tests/unity/unity.d \
./grapics/tests/unity/unity_support.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/tests/unity/%.o grapics/tests/unity/%.su grapics/tests/unity/%.cyclo: ../grapics/tests/unity/%.c grapics/tests/unity/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-tests-2f-unity

clean-grapics-2f-tests-2f-unity:
	-$(RM) ./grapics/tests/unity/unity.cyclo ./grapics/tests/unity/unity.d ./grapics/tests/unity/unity.o ./grapics/tests/unity/unity.su ./grapics/tests/unity/unity_support.cyclo ./grapics/tests/unity/unity_support.d ./grapics/tests/unity/unity_support.o ./grapics/tests/unity/unity_support.su

.PHONY: clean-grapics-2f-tests-2f-unity

