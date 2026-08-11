################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/src/draw/stm32_dma2d/lv_gpu_stm32_dma2d.c 

OBJS += \
./grapics/src/draw/stm32_dma2d/lv_gpu_stm32_dma2d.o 

C_DEPS += \
./grapics/src/draw/stm32_dma2d/lv_gpu_stm32_dma2d.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/src/draw/stm32_dma2d/%.o grapics/src/draw/stm32_dma2d/%.su grapics/src/draw/stm32_dma2d/%.cyclo: ../grapics/src/draw/stm32_dma2d/%.c grapics/src/draw/stm32_dma2d/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-src-2f-draw-2f-stm32_dma2d

clean-grapics-2f-src-2f-draw-2f-stm32_dma2d:
	-$(RM) ./grapics/src/draw/stm32_dma2d/lv_gpu_stm32_dma2d.cyclo ./grapics/src/draw/stm32_dma2d/lv_gpu_stm32_dma2d.d ./grapics/src/draw/stm32_dma2d/lv_gpu_stm32_dma2d.o ./grapics/src/draw/stm32_dma2d/lv_gpu_stm32_dma2d.su

.PHONY: clean-grapics-2f-src-2f-draw-2f-stm32_dma2d

