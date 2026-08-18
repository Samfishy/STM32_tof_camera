################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/src/draw/sw/lv_draw_sw.c \
../grapics/src/draw/sw/lv_draw_sw_arc.c \
../grapics/src/draw/sw/lv_draw_sw_blend.c \
../grapics/src/draw/sw/lv_draw_sw_dither.c \
../grapics/src/draw/sw/lv_draw_sw_gradient.c \
../grapics/src/draw/sw/lv_draw_sw_img.c \
../grapics/src/draw/sw/lv_draw_sw_layer.c \
../grapics/src/draw/sw/lv_draw_sw_letter.c \
../grapics/src/draw/sw/lv_draw_sw_line.c \
../grapics/src/draw/sw/lv_draw_sw_polygon.c \
../grapics/src/draw/sw/lv_draw_sw_rect.c \
../grapics/src/draw/sw/lv_draw_sw_transform.c 

OBJS += \
./grapics/src/draw/sw/lv_draw_sw.o \
./grapics/src/draw/sw/lv_draw_sw_arc.o \
./grapics/src/draw/sw/lv_draw_sw_blend.o \
./grapics/src/draw/sw/lv_draw_sw_dither.o \
./grapics/src/draw/sw/lv_draw_sw_gradient.o \
./grapics/src/draw/sw/lv_draw_sw_img.o \
./grapics/src/draw/sw/lv_draw_sw_layer.o \
./grapics/src/draw/sw/lv_draw_sw_letter.o \
./grapics/src/draw/sw/lv_draw_sw_line.o \
./grapics/src/draw/sw/lv_draw_sw_polygon.o \
./grapics/src/draw/sw/lv_draw_sw_rect.o \
./grapics/src/draw/sw/lv_draw_sw_transform.o 

C_DEPS += \
./grapics/src/draw/sw/lv_draw_sw.d \
./grapics/src/draw/sw/lv_draw_sw_arc.d \
./grapics/src/draw/sw/lv_draw_sw_blend.d \
./grapics/src/draw/sw/lv_draw_sw_dither.d \
./grapics/src/draw/sw/lv_draw_sw_gradient.d \
./grapics/src/draw/sw/lv_draw_sw_img.d \
./grapics/src/draw/sw/lv_draw_sw_layer.d \
./grapics/src/draw/sw/lv_draw_sw_letter.d \
./grapics/src/draw/sw/lv_draw_sw_line.d \
./grapics/src/draw/sw/lv_draw_sw_polygon.d \
./grapics/src/draw/sw/lv_draw_sw_rect.d \
./grapics/src/draw/sw/lv_draw_sw_transform.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/src/draw/sw/%.o grapics/src/draw/sw/%.su grapics/src/draw/sw/%.cyclo: ../grapics/src/draw/sw/%.c grapics/src/draw/sw/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-src-2f-draw-2f-sw

clean-grapics-2f-src-2f-draw-2f-sw:
	-$(RM) ./grapics/src/draw/sw/lv_draw_sw.cyclo ./grapics/src/draw/sw/lv_draw_sw.d ./grapics/src/draw/sw/lv_draw_sw.o ./grapics/src/draw/sw/lv_draw_sw.su ./grapics/src/draw/sw/lv_draw_sw_arc.cyclo ./grapics/src/draw/sw/lv_draw_sw_arc.d ./grapics/src/draw/sw/lv_draw_sw_arc.o ./grapics/src/draw/sw/lv_draw_sw_arc.su ./grapics/src/draw/sw/lv_draw_sw_blend.cyclo ./grapics/src/draw/sw/lv_draw_sw_blend.d ./grapics/src/draw/sw/lv_draw_sw_blend.o ./grapics/src/draw/sw/lv_draw_sw_blend.su ./grapics/src/draw/sw/lv_draw_sw_dither.cyclo ./grapics/src/draw/sw/lv_draw_sw_dither.d ./grapics/src/draw/sw/lv_draw_sw_dither.o ./grapics/src/draw/sw/lv_draw_sw_dither.su ./grapics/src/draw/sw/lv_draw_sw_gradient.cyclo ./grapics/src/draw/sw/lv_draw_sw_gradient.d ./grapics/src/draw/sw/lv_draw_sw_gradient.o ./grapics/src/draw/sw/lv_draw_sw_gradient.su ./grapics/src/draw/sw/lv_draw_sw_img.cyclo ./grapics/src/draw/sw/lv_draw_sw_img.d ./grapics/src/draw/sw/lv_draw_sw_img.o ./grapics/src/draw/sw/lv_draw_sw_img.su ./grapics/src/draw/sw/lv_draw_sw_layer.cyclo ./grapics/src/draw/sw/lv_draw_sw_layer.d ./grapics/src/draw/sw/lv_draw_sw_layer.o ./grapics/src/draw/sw/lv_draw_sw_layer.su ./grapics/src/draw/sw/lv_draw_sw_letter.cyclo ./grapics/src/draw/sw/lv_draw_sw_letter.d ./grapics/src/draw/sw/lv_draw_sw_letter.o ./grapics/src/draw/sw/lv_draw_sw_letter.su ./grapics/src/draw/sw/lv_draw_sw_line.cyclo ./grapics/src/draw/sw/lv_draw_sw_line.d ./grapics/src/draw/sw/lv_draw_sw_line.o ./grapics/src/draw/sw/lv_draw_sw_line.su ./grapics/src/draw/sw/lv_draw_sw_polygon.cyclo ./grapics/src/draw/sw/lv_draw_sw_polygon.d ./grapics/src/draw/sw/lv_draw_sw_polygon.o ./grapics/src/draw/sw/lv_draw_sw_polygon.su ./grapics/src/draw/sw/lv_draw_sw_rect.cyclo ./grapics/src/draw/sw/lv_draw_sw_rect.d ./grapics/src/draw/sw/lv_draw_sw_rect.o ./grapics/src/draw/sw/lv_draw_sw_rect.su ./grapics/src/draw/sw/lv_draw_sw_transform.cyclo ./grapics/src/draw/sw/lv_draw_sw_transform.d ./grapics/src/draw/sw/lv_draw_sw_transform.o ./grapics/src/draw/sw/lv_draw_sw_transform.su

.PHONY: clean-grapics-2f-src-2f-draw-2f-sw

