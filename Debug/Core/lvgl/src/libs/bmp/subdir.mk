################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lvgl/src/libs/bmp/lv_bmp.c 

OBJS += \
./Core/lvgl/src/libs/bmp/lv_bmp.o 

C_DEPS += \
./Core/lvgl/src/libs/bmp/lv_bmp.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lvgl/src/libs/bmp/%.o Core/lvgl/src/libs/bmp/%.su Core/lvgl/src/libs/bmp/%.cyclo: ../Core/lvgl/src/libs/bmp/%.c Core/lvgl/src/libs/bmp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/lvgl" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lvgl-2f-src-2f-libs-2f-bmp

clean-Core-2f-lvgl-2f-src-2f-libs-2f-bmp:
	-$(RM) ./Core/lvgl/src/libs/bmp/lv_bmp.cyclo ./Core/lvgl/src/libs/bmp/lv_bmp.d ./Core/lvgl/src/libs/bmp/lv_bmp.o ./Core/lvgl/src/libs/bmp/lv_bmp.su

.PHONY: clean-Core-2f-lvgl-2f-src-2f-libs-2f-bmp

