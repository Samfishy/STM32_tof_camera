################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/src/core/lv_disp.c \
../grapics/src/core/lv_event.c \
../grapics/src/core/lv_group.c \
../grapics/src/core/lv_indev.c \
../grapics/src/core/lv_indev_scroll.c \
../grapics/src/core/lv_obj.c \
../grapics/src/core/lv_obj_class.c \
../grapics/src/core/lv_obj_draw.c \
../grapics/src/core/lv_obj_pos.c \
../grapics/src/core/lv_obj_scroll.c \
../grapics/src/core/lv_obj_style.c \
../grapics/src/core/lv_obj_style_gen.c \
../grapics/src/core/lv_obj_tree.c \
../grapics/src/core/lv_refr.c \
../grapics/src/core/lv_theme.c 

OBJS += \
./grapics/src/core/lv_disp.o \
./grapics/src/core/lv_event.o \
./grapics/src/core/lv_group.o \
./grapics/src/core/lv_indev.o \
./grapics/src/core/lv_indev_scroll.o \
./grapics/src/core/lv_obj.o \
./grapics/src/core/lv_obj_class.o \
./grapics/src/core/lv_obj_draw.o \
./grapics/src/core/lv_obj_pos.o \
./grapics/src/core/lv_obj_scroll.o \
./grapics/src/core/lv_obj_style.o \
./grapics/src/core/lv_obj_style_gen.o \
./grapics/src/core/lv_obj_tree.o \
./grapics/src/core/lv_refr.o \
./grapics/src/core/lv_theme.o 

C_DEPS += \
./grapics/src/core/lv_disp.d \
./grapics/src/core/lv_event.d \
./grapics/src/core/lv_group.d \
./grapics/src/core/lv_indev.d \
./grapics/src/core/lv_indev_scroll.d \
./grapics/src/core/lv_obj.d \
./grapics/src/core/lv_obj_class.d \
./grapics/src/core/lv_obj_draw.d \
./grapics/src/core/lv_obj_pos.d \
./grapics/src/core/lv_obj_scroll.d \
./grapics/src/core/lv_obj_style.d \
./grapics/src/core/lv_obj_style_gen.d \
./grapics/src/core/lv_obj_tree.d \
./grapics/src/core/lv_refr.d \
./grapics/src/core/lv_theme.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/src/core/%.o grapics/src/core/%.su grapics/src/core/%.cyclo: ../grapics/src/core/%.c grapics/src/core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-src-2f-core

clean-grapics-2f-src-2f-core:
	-$(RM) ./grapics/src/core/lv_disp.cyclo ./grapics/src/core/lv_disp.d ./grapics/src/core/lv_disp.o ./grapics/src/core/lv_disp.su ./grapics/src/core/lv_event.cyclo ./grapics/src/core/lv_event.d ./grapics/src/core/lv_event.o ./grapics/src/core/lv_event.su ./grapics/src/core/lv_group.cyclo ./grapics/src/core/lv_group.d ./grapics/src/core/lv_group.o ./grapics/src/core/lv_group.su ./grapics/src/core/lv_indev.cyclo ./grapics/src/core/lv_indev.d ./grapics/src/core/lv_indev.o ./grapics/src/core/lv_indev.su ./grapics/src/core/lv_indev_scroll.cyclo ./grapics/src/core/lv_indev_scroll.d ./grapics/src/core/lv_indev_scroll.o ./grapics/src/core/lv_indev_scroll.su ./grapics/src/core/lv_obj.cyclo ./grapics/src/core/lv_obj.d ./grapics/src/core/lv_obj.o ./grapics/src/core/lv_obj.su ./grapics/src/core/lv_obj_class.cyclo ./grapics/src/core/lv_obj_class.d ./grapics/src/core/lv_obj_class.o ./grapics/src/core/lv_obj_class.su ./grapics/src/core/lv_obj_draw.cyclo ./grapics/src/core/lv_obj_draw.d ./grapics/src/core/lv_obj_draw.o ./grapics/src/core/lv_obj_draw.su ./grapics/src/core/lv_obj_pos.cyclo ./grapics/src/core/lv_obj_pos.d ./grapics/src/core/lv_obj_pos.o ./grapics/src/core/lv_obj_pos.su ./grapics/src/core/lv_obj_scroll.cyclo ./grapics/src/core/lv_obj_scroll.d ./grapics/src/core/lv_obj_scroll.o ./grapics/src/core/lv_obj_scroll.su ./grapics/src/core/lv_obj_style.cyclo ./grapics/src/core/lv_obj_style.d ./grapics/src/core/lv_obj_style.o ./grapics/src/core/lv_obj_style.su ./grapics/src/core/lv_obj_style_gen.cyclo ./grapics/src/core/lv_obj_style_gen.d ./grapics/src/core/lv_obj_style_gen.o ./grapics/src/core/lv_obj_style_gen.su ./grapics/src/core/lv_obj_tree.cyclo ./grapics/src/core/lv_obj_tree.d ./grapics/src/core/lv_obj_tree.o ./grapics/src/core/lv_obj_tree.su ./grapics/src/core/lv_refr.cyclo ./grapics/src/core/lv_refr.d ./grapics/src/core/lv_refr.o ./grapics/src/core/lv_refr.su ./grapics/src/core/lv_theme.cyclo ./grapics/src/core/lv_theme.d ./grapics/src/core/lv_theme.o ./grapics/src/core/lv_theme.su

.PHONY: clean-grapics-2f-src-2f-core

