################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/src/draw/sdl/lv_draw_sdl.c \
../grapics/src/draw/sdl/lv_draw_sdl_arc.c \
../grapics/src/draw/sdl/lv_draw_sdl_bg.c \
../grapics/src/draw/sdl/lv_draw_sdl_composite.c \
../grapics/src/draw/sdl/lv_draw_sdl_img.c \
../grapics/src/draw/sdl/lv_draw_sdl_label.c \
../grapics/src/draw/sdl/lv_draw_sdl_layer.c \
../grapics/src/draw/sdl/lv_draw_sdl_line.c \
../grapics/src/draw/sdl/lv_draw_sdl_mask.c \
../grapics/src/draw/sdl/lv_draw_sdl_polygon.c \
../grapics/src/draw/sdl/lv_draw_sdl_rect.c \
../grapics/src/draw/sdl/lv_draw_sdl_stack_blur.c \
../grapics/src/draw/sdl/lv_draw_sdl_texture_cache.c \
../grapics/src/draw/sdl/lv_draw_sdl_utils.c 

OBJS += \
./grapics/src/draw/sdl/lv_draw_sdl.o \
./grapics/src/draw/sdl/lv_draw_sdl_arc.o \
./grapics/src/draw/sdl/lv_draw_sdl_bg.o \
./grapics/src/draw/sdl/lv_draw_sdl_composite.o \
./grapics/src/draw/sdl/lv_draw_sdl_img.o \
./grapics/src/draw/sdl/lv_draw_sdl_label.o \
./grapics/src/draw/sdl/lv_draw_sdl_layer.o \
./grapics/src/draw/sdl/lv_draw_sdl_line.o \
./grapics/src/draw/sdl/lv_draw_sdl_mask.o \
./grapics/src/draw/sdl/lv_draw_sdl_polygon.o \
./grapics/src/draw/sdl/lv_draw_sdl_rect.o \
./grapics/src/draw/sdl/lv_draw_sdl_stack_blur.o \
./grapics/src/draw/sdl/lv_draw_sdl_texture_cache.o \
./grapics/src/draw/sdl/lv_draw_sdl_utils.o 

C_DEPS += \
./grapics/src/draw/sdl/lv_draw_sdl.d \
./grapics/src/draw/sdl/lv_draw_sdl_arc.d \
./grapics/src/draw/sdl/lv_draw_sdl_bg.d \
./grapics/src/draw/sdl/lv_draw_sdl_composite.d \
./grapics/src/draw/sdl/lv_draw_sdl_img.d \
./grapics/src/draw/sdl/lv_draw_sdl_label.d \
./grapics/src/draw/sdl/lv_draw_sdl_layer.d \
./grapics/src/draw/sdl/lv_draw_sdl_line.d \
./grapics/src/draw/sdl/lv_draw_sdl_mask.d \
./grapics/src/draw/sdl/lv_draw_sdl_polygon.d \
./grapics/src/draw/sdl/lv_draw_sdl_rect.d \
./grapics/src/draw/sdl/lv_draw_sdl_stack_blur.d \
./grapics/src/draw/sdl/lv_draw_sdl_texture_cache.d \
./grapics/src/draw/sdl/lv_draw_sdl_utils.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/src/draw/sdl/%.o grapics/src/draw/sdl/%.su grapics/src/draw/sdl/%.cyclo: ../grapics/src/draw/sdl/%.c grapics/src/draw/sdl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-src-2f-draw-2f-sdl

clean-grapics-2f-src-2f-draw-2f-sdl:
	-$(RM) ./grapics/src/draw/sdl/lv_draw_sdl.cyclo ./grapics/src/draw/sdl/lv_draw_sdl.d ./grapics/src/draw/sdl/lv_draw_sdl.o ./grapics/src/draw/sdl/lv_draw_sdl.su ./grapics/src/draw/sdl/lv_draw_sdl_arc.cyclo ./grapics/src/draw/sdl/lv_draw_sdl_arc.d ./grapics/src/draw/sdl/lv_draw_sdl_arc.o ./grapics/src/draw/sdl/lv_draw_sdl_arc.su ./grapics/src/draw/sdl/lv_draw_sdl_bg.cyclo ./grapics/src/draw/sdl/lv_draw_sdl_bg.d ./grapics/src/draw/sdl/lv_draw_sdl_bg.o ./grapics/src/draw/sdl/lv_draw_sdl_bg.su ./grapics/src/draw/sdl/lv_draw_sdl_composite.cyclo ./grapics/src/draw/sdl/lv_draw_sdl_composite.d ./grapics/src/draw/sdl/lv_draw_sdl_composite.o ./grapics/src/draw/sdl/lv_draw_sdl_composite.su ./grapics/src/draw/sdl/lv_draw_sdl_img.cyclo ./grapics/src/draw/sdl/lv_draw_sdl_img.d ./grapics/src/draw/sdl/lv_draw_sdl_img.o ./grapics/src/draw/sdl/lv_draw_sdl_img.su ./grapics/src/draw/sdl/lv_draw_sdl_label.cyclo ./grapics/src/draw/sdl/lv_draw_sdl_label.d ./grapics/src/draw/sdl/lv_draw_sdl_label.o ./grapics/src/draw/sdl/lv_draw_sdl_label.su ./grapics/src/draw/sdl/lv_draw_sdl_layer.cyclo ./grapics/src/draw/sdl/lv_draw_sdl_layer.d ./grapics/src/draw/sdl/lv_draw_sdl_layer.o ./grapics/src/draw/sdl/lv_draw_sdl_layer.su ./grapics/src/draw/sdl/lv_draw_sdl_line.cyclo ./grapics/src/draw/sdl/lv_draw_sdl_line.d ./grapics/src/draw/sdl/lv_draw_sdl_line.o ./grapics/src/draw/sdl/lv_draw_sdl_line.su ./grapics/src/draw/sdl/lv_draw_sdl_mask.cyclo ./grapics/src/draw/sdl/lv_draw_sdl_mask.d ./grapics/src/draw/sdl/lv_draw_sdl_mask.o ./grapics/src/draw/sdl/lv_draw_sdl_mask.su ./grapics/src/draw/sdl/lv_draw_sdl_polygon.cyclo ./grapics/src/draw/sdl/lv_draw_sdl_polygon.d ./grapics/src/draw/sdl/lv_draw_sdl_polygon.o ./grapics/src/draw/sdl/lv_draw_sdl_polygon.su ./grapics/src/draw/sdl/lv_draw_sdl_rect.cyclo ./grapics/src/draw/sdl/lv_draw_sdl_rect.d ./grapics/src/draw/sdl/lv_draw_sdl_rect.o ./grapics/src/draw/sdl/lv_draw_sdl_rect.su ./grapics/src/draw/sdl/lv_draw_sdl_stack_blur.cyclo ./grapics/src/draw/sdl/lv_draw_sdl_stack_blur.d ./grapics/src/draw/sdl/lv_draw_sdl_stack_blur.o ./grapics/src/draw/sdl/lv_draw_sdl_stack_blur.su ./grapics/src/draw/sdl/lv_draw_sdl_texture_cache.cyclo ./grapics/src/draw/sdl/lv_draw_sdl_texture_cache.d ./grapics/src/draw/sdl/lv_draw_sdl_texture_cache.o ./grapics/src/draw/sdl/lv_draw_sdl_texture_cache.su ./grapics/src/draw/sdl/lv_draw_sdl_utils.cyclo ./grapics/src/draw/sdl/lv_draw_sdl_utils.d ./grapics/src/draw/sdl/lv_draw_sdl_utils.o ./grapics/src/draw/sdl/lv_draw_sdl_utils.su

.PHONY: clean-grapics-2f-src-2f-draw-2f-sdl

