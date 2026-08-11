################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lvgl/tests/src/test_libs/rnd_unicodes/lv_rnd_unicodes.c 

OBJS += \
./Core/lvgl/tests/src/test_libs/rnd_unicodes/lv_rnd_unicodes.o 

C_DEPS += \
./Core/lvgl/tests/src/test_libs/rnd_unicodes/lv_rnd_unicodes.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lvgl/tests/src/test_libs/rnd_unicodes/%.o Core/lvgl/tests/src/test_libs/rnd_unicodes/%.su Core/lvgl/tests/src/test_libs/rnd_unicodes/%.cyclo: ../Core/lvgl/tests/src/test_libs/rnd_unicodes/%.c Core/lvgl/tests/src/test_libs/rnd_unicodes/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/lvgl" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lvgl-2f-tests-2f-src-2f-test_libs-2f-rnd_unicodes

clean-Core-2f-lvgl-2f-tests-2f-src-2f-test_libs-2f-rnd_unicodes:
	-$(RM) ./Core/lvgl/tests/src/test_libs/rnd_unicodes/lv_rnd_unicodes.cyclo ./Core/lvgl/tests/src/test_libs/rnd_unicodes/lv_rnd_unicodes.d ./Core/lvgl/tests/src/test_libs/rnd_unicodes/lv_rnd_unicodes.o ./Core/lvgl/tests/src/test_libs/rnd_unicodes/lv_rnd_unicodes.su

.PHONY: clean-Core-2f-lvgl-2f-tests-2f-src-2f-test_libs-2f-rnd_unicodes

