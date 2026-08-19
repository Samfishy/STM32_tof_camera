################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/src/draw/nema_gfx/lv_draw_nema_gfx.c \
../grapics/src/draw/nema_gfx/lv_draw_nema_gfx_img.c \
../grapics/src/draw/nema_gfx/lv_draw_nema_gfx_letter.c 

OBJS += \
./grapics/src/draw/nema_gfx/lv_draw_nema_gfx.o \
./grapics/src/draw/nema_gfx/lv_draw_nema_gfx_img.o \
./grapics/src/draw/nema_gfx/lv_draw_nema_gfx_letter.o 

C_DEPS += \
./grapics/src/draw/nema_gfx/lv_draw_nema_gfx.d \
./grapics/src/draw/nema_gfx/lv_draw_nema_gfx_img.d \
./grapics/src/draw/nema_gfx/lv_draw_nema_gfx_letter.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/src/draw/nema_gfx/%.o grapics/src/draw/nema_gfx/%.su grapics/src/draw/nema_gfx/%.cyclo: ../grapics/src/draw/nema_gfx/%.c grapics/src/draw/nema_gfx/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-src-2f-draw-2f-nema_gfx

clean-grapics-2f-src-2f-draw-2f-nema_gfx:
	-$(RM) ./grapics/src/draw/nema_gfx/lv_draw_nema_gfx.cyclo ./grapics/src/draw/nema_gfx/lv_draw_nema_gfx.d ./grapics/src/draw/nema_gfx/lv_draw_nema_gfx.o ./grapics/src/draw/nema_gfx/lv_draw_nema_gfx.su ./grapics/src/draw/nema_gfx/lv_draw_nema_gfx_img.cyclo ./grapics/src/draw/nema_gfx/lv_draw_nema_gfx_img.d ./grapics/src/draw/nema_gfx/lv_draw_nema_gfx_img.o ./grapics/src/draw/nema_gfx/lv_draw_nema_gfx_img.su ./grapics/src/draw/nema_gfx/lv_draw_nema_gfx_letter.cyclo ./grapics/src/draw/nema_gfx/lv_draw_nema_gfx_letter.d ./grapics/src/draw/nema_gfx/lv_draw_nema_gfx_letter.o ./grapics/src/draw/nema_gfx/lv_draw_nema_gfx_letter.su

.PHONY: clean-grapics-2f-src-2f-draw-2f-nema_gfx

