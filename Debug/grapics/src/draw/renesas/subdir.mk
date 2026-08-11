################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/src/draw/renesas/lv_gpu_d2_draw_label.c \
../grapics/src/draw/renesas/lv_gpu_d2_ra6m3.c 

OBJS += \
./grapics/src/draw/renesas/lv_gpu_d2_draw_label.o \
./grapics/src/draw/renesas/lv_gpu_d2_ra6m3.o 

C_DEPS += \
./grapics/src/draw/renesas/lv_gpu_d2_draw_label.d \
./grapics/src/draw/renesas/lv_gpu_d2_ra6m3.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/src/draw/renesas/%.o grapics/src/draw/renesas/%.su grapics/src/draw/renesas/%.cyclo: ../grapics/src/draw/renesas/%.c grapics/src/draw/renesas/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-src-2f-draw-2f-renesas

clean-grapics-2f-src-2f-draw-2f-renesas:
	-$(RM) ./grapics/src/draw/renesas/lv_gpu_d2_draw_label.cyclo ./grapics/src/draw/renesas/lv_gpu_d2_draw_label.d ./grapics/src/draw/renesas/lv_gpu_d2_draw_label.o ./grapics/src/draw/renesas/lv_gpu_d2_draw_label.su ./grapics/src/draw/renesas/lv_gpu_d2_ra6m3.cyclo ./grapics/src/draw/renesas/lv_gpu_d2_ra6m3.d ./grapics/src/draw/renesas/lv_gpu_d2_ra6m3.o ./grapics/src/draw/renesas/lv_gpu_d2_ra6m3.su

.PHONY: clean-grapics-2f-src-2f-draw-2f-renesas

