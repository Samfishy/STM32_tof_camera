################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lvgl/src/widgets/slider/lv_slider.c 

OBJS += \
./Core/lvgl/src/widgets/slider/lv_slider.o 

C_DEPS += \
./Core/lvgl/src/widgets/slider/lv_slider.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lvgl/src/widgets/slider/%.o Core/lvgl/src/widgets/slider/%.su Core/lvgl/src/widgets/slider/%.cyclo: ../Core/lvgl/src/widgets/slider/%.c Core/lvgl/src/widgets/slider/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/lvgl" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lvgl-2f-src-2f-widgets-2f-slider

clean-Core-2f-lvgl-2f-src-2f-widgets-2f-slider:
	-$(RM) ./Core/lvgl/src/widgets/slider/lv_slider.cyclo ./Core/lvgl/src/widgets/slider/lv_slider.d ./Core/lvgl/src/widgets/slider/lv_slider.o ./Core/lvgl/src/widgets/slider/lv_slider.su

.PHONY: clean-Core-2f-lvgl-2f-src-2f-widgets-2f-slider

