################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/src/draw/nxp/vglite/lv_draw_vglite.c \
../grapics/src/draw/nxp/vglite/lv_draw_vglite_arc.c \
../grapics/src/draw/nxp/vglite/lv_draw_vglite_blend.c \
../grapics/src/draw/nxp/vglite/lv_draw_vglite_line.c \
../grapics/src/draw/nxp/vglite/lv_draw_vglite_rect.c \
../grapics/src/draw/nxp/vglite/lv_vglite_buf.c \
../grapics/src/draw/nxp/vglite/lv_vglite_utils.c 

OBJS += \
./grapics/src/draw/nxp/vglite/lv_draw_vglite.o \
./grapics/src/draw/nxp/vglite/lv_draw_vglite_arc.o \
./grapics/src/draw/nxp/vglite/lv_draw_vglite_blend.o \
./grapics/src/draw/nxp/vglite/lv_draw_vglite_line.o \
./grapics/src/draw/nxp/vglite/lv_draw_vglite_rect.o \
./grapics/src/draw/nxp/vglite/lv_vglite_buf.o \
./grapics/src/draw/nxp/vglite/lv_vglite_utils.o 

C_DEPS += \
./grapics/src/draw/nxp/vglite/lv_draw_vglite.d \
./grapics/src/draw/nxp/vglite/lv_draw_vglite_arc.d \
./grapics/src/draw/nxp/vglite/lv_draw_vglite_blend.d \
./grapics/src/draw/nxp/vglite/lv_draw_vglite_line.d \
./grapics/src/draw/nxp/vglite/lv_draw_vglite_rect.d \
./grapics/src/draw/nxp/vglite/lv_vglite_buf.d \
./grapics/src/draw/nxp/vglite/lv_vglite_utils.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/src/draw/nxp/vglite/%.o grapics/src/draw/nxp/vglite/%.su grapics/src/draw/nxp/vglite/%.cyclo: ../grapics/src/draw/nxp/vglite/%.c grapics/src/draw/nxp/vglite/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-src-2f-draw-2f-nxp-2f-vglite

clean-grapics-2f-src-2f-draw-2f-nxp-2f-vglite:
	-$(RM) ./grapics/src/draw/nxp/vglite/lv_draw_vglite.cyclo ./grapics/src/draw/nxp/vglite/lv_draw_vglite.d ./grapics/src/draw/nxp/vglite/lv_draw_vglite.o ./grapics/src/draw/nxp/vglite/lv_draw_vglite.su ./grapics/src/draw/nxp/vglite/lv_draw_vglite_arc.cyclo ./grapics/src/draw/nxp/vglite/lv_draw_vglite_arc.d ./grapics/src/draw/nxp/vglite/lv_draw_vglite_arc.o ./grapics/src/draw/nxp/vglite/lv_draw_vglite_arc.su ./grapics/src/draw/nxp/vglite/lv_draw_vglite_blend.cyclo ./grapics/src/draw/nxp/vglite/lv_draw_vglite_blend.d ./grapics/src/draw/nxp/vglite/lv_draw_vglite_blend.o ./grapics/src/draw/nxp/vglite/lv_draw_vglite_blend.su ./grapics/src/draw/nxp/vglite/lv_draw_vglite_line.cyclo ./grapics/src/draw/nxp/vglite/lv_draw_vglite_line.d ./grapics/src/draw/nxp/vglite/lv_draw_vglite_line.o ./grapics/src/draw/nxp/vglite/lv_draw_vglite_line.su ./grapics/src/draw/nxp/vglite/lv_draw_vglite_rect.cyclo ./grapics/src/draw/nxp/vglite/lv_draw_vglite_rect.d ./grapics/src/draw/nxp/vglite/lv_draw_vglite_rect.o ./grapics/src/draw/nxp/vglite/lv_draw_vglite_rect.su ./grapics/src/draw/nxp/vglite/lv_vglite_buf.cyclo ./grapics/src/draw/nxp/vglite/lv_vglite_buf.d ./grapics/src/draw/nxp/vglite/lv_vglite_buf.o ./grapics/src/draw/nxp/vglite/lv_vglite_buf.su ./grapics/src/draw/nxp/vglite/lv_vglite_utils.cyclo ./grapics/src/draw/nxp/vglite/lv_vglite_utils.d ./grapics/src/draw/nxp/vglite/lv_vglite_utils.o ./grapics/src/draw/nxp/vglite/lv_vglite_utils.su

.PHONY: clean-grapics-2f-src-2f-draw-2f-nxp-2f-vglite

