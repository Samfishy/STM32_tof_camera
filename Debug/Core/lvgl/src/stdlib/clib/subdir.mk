################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lvgl/src/stdlib/clib/lv_mem_core_clib.c \
../Core/lvgl/src/stdlib/clib/lv_sprintf_clib.c \
../Core/lvgl/src/stdlib/clib/lv_string_clib.c 

OBJS += \
./Core/lvgl/src/stdlib/clib/lv_mem_core_clib.o \
./Core/lvgl/src/stdlib/clib/lv_sprintf_clib.o \
./Core/lvgl/src/stdlib/clib/lv_string_clib.o 

C_DEPS += \
./Core/lvgl/src/stdlib/clib/lv_mem_core_clib.d \
./Core/lvgl/src/stdlib/clib/lv_sprintf_clib.d \
./Core/lvgl/src/stdlib/clib/lv_string_clib.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lvgl/src/stdlib/clib/%.o Core/lvgl/src/stdlib/clib/%.su Core/lvgl/src/stdlib/clib/%.cyclo: ../Core/lvgl/src/stdlib/clib/%.c Core/lvgl/src/stdlib/clib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/lvgl" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lvgl-2f-src-2f-stdlib-2f-clib

clean-Core-2f-lvgl-2f-src-2f-stdlib-2f-clib:
	-$(RM) ./Core/lvgl/src/stdlib/clib/lv_mem_core_clib.cyclo ./Core/lvgl/src/stdlib/clib/lv_mem_core_clib.d ./Core/lvgl/src/stdlib/clib/lv_mem_core_clib.o ./Core/lvgl/src/stdlib/clib/lv_mem_core_clib.su ./Core/lvgl/src/stdlib/clib/lv_sprintf_clib.cyclo ./Core/lvgl/src/stdlib/clib/lv_sprintf_clib.d ./Core/lvgl/src/stdlib/clib/lv_sprintf_clib.o ./Core/lvgl/src/stdlib/clib/lv_sprintf_clib.su ./Core/lvgl/src/stdlib/clib/lv_string_clib.cyclo ./Core/lvgl/src/stdlib/clib/lv_string_clib.d ./Core/lvgl/src/stdlib/clib/lv_string_clib.o ./Core/lvgl/src/stdlib/clib/lv_string_clib.su

.PHONY: clean-Core-2f-lvgl-2f-src-2f-stdlib-2f-clib

