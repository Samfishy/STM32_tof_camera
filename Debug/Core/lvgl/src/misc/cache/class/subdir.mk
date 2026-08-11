################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lvgl/src/misc/cache/class/lv_cache_lru_ll.c \
../Core/lvgl/src/misc/cache/class/lv_cache_lru_rb.c \
../Core/lvgl/src/misc/cache/class/lv_cache_sc_da.c 

OBJS += \
./Core/lvgl/src/misc/cache/class/lv_cache_lru_ll.o \
./Core/lvgl/src/misc/cache/class/lv_cache_lru_rb.o \
./Core/lvgl/src/misc/cache/class/lv_cache_sc_da.o 

C_DEPS += \
./Core/lvgl/src/misc/cache/class/lv_cache_lru_ll.d \
./Core/lvgl/src/misc/cache/class/lv_cache_lru_rb.d \
./Core/lvgl/src/misc/cache/class/lv_cache_sc_da.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lvgl/src/misc/cache/class/%.o Core/lvgl/src/misc/cache/class/%.su Core/lvgl/src/misc/cache/class/%.cyclo: ../Core/lvgl/src/misc/cache/class/%.c Core/lvgl/src/misc/cache/class/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/lvgl" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lvgl-2f-src-2f-misc-2f-cache-2f-class

clean-Core-2f-lvgl-2f-src-2f-misc-2f-cache-2f-class:
	-$(RM) ./Core/lvgl/src/misc/cache/class/lv_cache_lru_ll.cyclo ./Core/lvgl/src/misc/cache/class/lv_cache_lru_ll.d ./Core/lvgl/src/misc/cache/class/lv_cache_lru_ll.o ./Core/lvgl/src/misc/cache/class/lv_cache_lru_ll.su ./Core/lvgl/src/misc/cache/class/lv_cache_lru_rb.cyclo ./Core/lvgl/src/misc/cache/class/lv_cache_lru_rb.d ./Core/lvgl/src/misc/cache/class/lv_cache_lru_rb.o ./Core/lvgl/src/misc/cache/class/lv_cache_lru_rb.su ./Core/lvgl/src/misc/cache/class/lv_cache_sc_da.cyclo ./Core/lvgl/src/misc/cache/class/lv_cache_sc_da.d ./Core/lvgl/src/misc/cache/class/lv_cache_sc_da.o ./Core/lvgl/src/misc/cache/class/lv_cache_sc_da.su

.PHONY: clean-Core-2f-lvgl-2f-src-2f-misc-2f-cache-2f-class

