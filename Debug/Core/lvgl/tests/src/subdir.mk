################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lvgl/tests/src/lv_test_init.c \
../Core/lvgl/tests/src/test_layout_switch.c 

OBJS += \
./Core/lvgl/tests/src/lv_test_init.o \
./Core/lvgl/tests/src/test_layout_switch.o 

C_DEPS += \
./Core/lvgl/tests/src/lv_test_init.d \
./Core/lvgl/tests/src/test_layout_switch.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lvgl/tests/src/%.o Core/lvgl/tests/src/%.su Core/lvgl/tests/src/%.cyclo: ../Core/lvgl/tests/src/%.c Core/lvgl/tests/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/lvgl" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lvgl-2f-tests-2f-src

clean-Core-2f-lvgl-2f-tests-2f-src:
	-$(RM) ./Core/lvgl/tests/src/lv_test_init.cyclo ./Core/lvgl/tests/src/lv_test_init.d ./Core/lvgl/tests/src/lv_test_init.o ./Core/lvgl/tests/src/lv_test_init.su ./Core/lvgl/tests/src/test_layout_switch.cyclo ./Core/lvgl/tests/src/test_layout_switch.d ./Core/lvgl/tests/src/test_layout_switch.o ./Core/lvgl/tests/src/test_layout_switch.su

.PHONY: clean-Core-2f-lvgl-2f-tests-2f-src

