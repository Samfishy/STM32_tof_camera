################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lvgl/src/font/font_manager/lv_font_manager.c \
../Core/lvgl/src/font/font_manager/lv_font_manager_recycle.c 

OBJS += \
./Core/lvgl/src/font/font_manager/lv_font_manager.o \
./Core/lvgl/src/font/font_manager/lv_font_manager_recycle.o 

C_DEPS += \
./Core/lvgl/src/font/font_manager/lv_font_manager.d \
./Core/lvgl/src/font/font_manager/lv_font_manager_recycle.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lvgl/src/font/font_manager/%.o Core/lvgl/src/font/font_manager/%.su Core/lvgl/src/font/font_manager/%.cyclo: ../Core/lvgl/src/font/font_manager/%.c Core/lvgl/src/font/font_manager/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/lvgl" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lvgl-2f-src-2f-font-2f-font_manager

clean-Core-2f-lvgl-2f-src-2f-font-2f-font_manager:
	-$(RM) ./Core/lvgl/src/font/font_manager/lv_font_manager.cyclo ./Core/lvgl/src/font/font_manager/lv_font_manager.d ./Core/lvgl/src/font/font_manager/lv_font_manager.o ./Core/lvgl/src/font/font_manager/lv_font_manager.su ./Core/lvgl/src/font/font_manager/lv_font_manager_recycle.cyclo ./Core/lvgl/src/font/font_manager/lv_font_manager_recycle.d ./Core/lvgl/src/font/font_manager/lv_font_manager_recycle.o ./Core/lvgl/src/font/font_manager/lv_font_manager_recycle.su

.PHONY: clean-Core-2f-lvgl-2f-src-2f-font-2f-font_manager

