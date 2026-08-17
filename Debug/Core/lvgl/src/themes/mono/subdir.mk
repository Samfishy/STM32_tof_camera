################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lvgl/src/themes/mono/lv_theme_mono.c 

OBJS += \
./Core/lvgl/src/themes/mono/lv_theme_mono.o 

C_DEPS += \
./Core/lvgl/src/themes/mono/lv_theme_mono.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lvgl/src/themes/mono/%.o Core/lvgl/src/themes/mono/%.su Core/lvgl/src/themes/mono/%.cyclo: ../Core/lvgl/src/themes/mono/%.c Core/lvgl/src/themes/mono/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/lvgl" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lvgl-2f-src-2f-themes-2f-mono

clean-Core-2f-lvgl-2f-src-2f-themes-2f-mono:
	-$(RM) ./Core/lvgl/src/themes/mono/lv_theme_mono.cyclo ./Core/lvgl/src/themes/mono/lv_theme_mono.d ./Core/lvgl/src/themes/mono/lv_theme_mono.o ./Core/lvgl/src/themes/mono/lv_theme_mono.su

.PHONY: clean-Core-2f-lvgl-2f-src-2f-themes-2f-mono

