################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/src/draw/lv_draw.c \
../grapics/src/draw/lv_draw_arc.c \
../grapics/src/draw/lv_draw_img.c \
../grapics/src/draw/lv_draw_label.c \
../grapics/src/draw/lv_draw_layer.c \
../grapics/src/draw/lv_draw_line.c \
../grapics/src/draw/lv_draw_mask.c \
../grapics/src/draw/lv_draw_rect.c \
../grapics/src/draw/lv_draw_transform.c \
../grapics/src/draw/lv_draw_triangle.c \
../grapics/src/draw/lv_img_buf.c \
../grapics/src/draw/lv_img_cache.c \
../grapics/src/draw/lv_img_decoder.c 

OBJS += \
./grapics/src/draw/lv_draw.o \
./grapics/src/draw/lv_draw_arc.o \
./grapics/src/draw/lv_draw_img.o \
./grapics/src/draw/lv_draw_label.o \
./grapics/src/draw/lv_draw_layer.o \
./grapics/src/draw/lv_draw_line.o \
./grapics/src/draw/lv_draw_mask.o \
./grapics/src/draw/lv_draw_rect.o \
./grapics/src/draw/lv_draw_transform.o \
./grapics/src/draw/lv_draw_triangle.o \
./grapics/src/draw/lv_img_buf.o \
./grapics/src/draw/lv_img_cache.o \
./grapics/src/draw/lv_img_decoder.o 

C_DEPS += \
./grapics/src/draw/lv_draw.d \
./grapics/src/draw/lv_draw_arc.d \
./grapics/src/draw/lv_draw_img.d \
./grapics/src/draw/lv_draw_label.d \
./grapics/src/draw/lv_draw_layer.d \
./grapics/src/draw/lv_draw_line.d \
./grapics/src/draw/lv_draw_mask.d \
./grapics/src/draw/lv_draw_rect.d \
./grapics/src/draw/lv_draw_transform.d \
./grapics/src/draw/lv_draw_triangle.d \
./grapics/src/draw/lv_img_buf.d \
./grapics/src/draw/lv_img_cache.d \
./grapics/src/draw/lv_img_decoder.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/src/draw/%.o grapics/src/draw/%.su grapics/src/draw/%.cyclo: ../grapics/src/draw/%.c grapics/src/draw/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-src-2f-draw

clean-grapics-2f-src-2f-draw:
	-$(RM) ./grapics/src/draw/lv_draw.cyclo ./grapics/src/draw/lv_draw.d ./grapics/src/draw/lv_draw.o ./grapics/src/draw/lv_draw.su ./grapics/src/draw/lv_draw_arc.cyclo ./grapics/src/draw/lv_draw_arc.d ./grapics/src/draw/lv_draw_arc.o ./grapics/src/draw/lv_draw_arc.su ./grapics/src/draw/lv_draw_img.cyclo ./grapics/src/draw/lv_draw_img.d ./grapics/src/draw/lv_draw_img.o ./grapics/src/draw/lv_draw_img.su ./grapics/src/draw/lv_draw_label.cyclo ./grapics/src/draw/lv_draw_label.d ./grapics/src/draw/lv_draw_label.o ./grapics/src/draw/lv_draw_label.su ./grapics/src/draw/lv_draw_layer.cyclo ./grapics/src/draw/lv_draw_layer.d ./grapics/src/draw/lv_draw_layer.o ./grapics/src/draw/lv_draw_layer.su ./grapics/src/draw/lv_draw_line.cyclo ./grapics/src/draw/lv_draw_line.d ./grapics/src/draw/lv_draw_line.o ./grapics/src/draw/lv_draw_line.su ./grapics/src/draw/lv_draw_mask.cyclo ./grapics/src/draw/lv_draw_mask.d ./grapics/src/draw/lv_draw_mask.o ./grapics/src/draw/lv_draw_mask.su ./grapics/src/draw/lv_draw_rect.cyclo ./grapics/src/draw/lv_draw_rect.d ./grapics/src/draw/lv_draw_rect.o ./grapics/src/draw/lv_draw_rect.su ./grapics/src/draw/lv_draw_transform.cyclo ./grapics/src/draw/lv_draw_transform.d ./grapics/src/draw/lv_draw_transform.o ./grapics/src/draw/lv_draw_transform.su ./grapics/src/draw/lv_draw_triangle.cyclo ./grapics/src/draw/lv_draw_triangle.d ./grapics/src/draw/lv_draw_triangle.o ./grapics/src/draw/lv_draw_triangle.su ./grapics/src/draw/lv_img_buf.cyclo ./grapics/src/draw/lv_img_buf.d ./grapics/src/draw/lv_img_buf.o ./grapics/src/draw/lv_img_buf.su ./grapics/src/draw/lv_img_cache.cyclo ./grapics/src/draw/lv_img_cache.d ./grapics/src/draw/lv_img_cache.o ./grapics/src/draw/lv_img_cache.su ./grapics/src/draw/lv_img_decoder.cyclo ./grapics/src/draw/lv_img_decoder.d ./grapics/src/draw/lv_img_decoder.o ./grapics/src/draw/lv_img_decoder.su

.PHONY: clean-grapics-2f-src-2f-draw

