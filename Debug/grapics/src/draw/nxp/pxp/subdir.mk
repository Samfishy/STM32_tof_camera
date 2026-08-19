################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/src/draw/nxp/pxp/lv_draw_pxp.c \
../grapics/src/draw/nxp/pxp/lv_draw_pxp_blend.c \
../grapics/src/draw/nxp/pxp/lv_gpu_nxp_pxp.c \
../grapics/src/draw/nxp/pxp/lv_gpu_nxp_pxp_osa.c 

OBJS += \
./grapics/src/draw/nxp/pxp/lv_draw_pxp.o \
./grapics/src/draw/nxp/pxp/lv_draw_pxp_blend.o \
./grapics/src/draw/nxp/pxp/lv_gpu_nxp_pxp.o \
./grapics/src/draw/nxp/pxp/lv_gpu_nxp_pxp_osa.o 

C_DEPS += \
./grapics/src/draw/nxp/pxp/lv_draw_pxp.d \
./grapics/src/draw/nxp/pxp/lv_draw_pxp_blend.d \
./grapics/src/draw/nxp/pxp/lv_gpu_nxp_pxp.d \
./grapics/src/draw/nxp/pxp/lv_gpu_nxp_pxp_osa.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/src/draw/nxp/pxp/%.o grapics/src/draw/nxp/pxp/%.su grapics/src/draw/nxp/pxp/%.cyclo: ../grapics/src/draw/nxp/pxp/%.c grapics/src/draw/nxp/pxp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-src-2f-draw-2f-nxp-2f-pxp

clean-grapics-2f-src-2f-draw-2f-nxp-2f-pxp:
	-$(RM) ./grapics/src/draw/nxp/pxp/lv_draw_pxp.cyclo ./grapics/src/draw/nxp/pxp/lv_draw_pxp.d ./grapics/src/draw/nxp/pxp/lv_draw_pxp.o ./grapics/src/draw/nxp/pxp/lv_draw_pxp.su ./grapics/src/draw/nxp/pxp/lv_draw_pxp_blend.cyclo ./grapics/src/draw/nxp/pxp/lv_draw_pxp_blend.d ./grapics/src/draw/nxp/pxp/lv_draw_pxp_blend.o ./grapics/src/draw/nxp/pxp/lv_draw_pxp_blend.su ./grapics/src/draw/nxp/pxp/lv_gpu_nxp_pxp.cyclo ./grapics/src/draw/nxp/pxp/lv_gpu_nxp_pxp.d ./grapics/src/draw/nxp/pxp/lv_gpu_nxp_pxp.o ./grapics/src/draw/nxp/pxp/lv_gpu_nxp_pxp.su ./grapics/src/draw/nxp/pxp/lv_gpu_nxp_pxp_osa.cyclo ./grapics/src/draw/nxp/pxp/lv_gpu_nxp_pxp_osa.d ./grapics/src/draw/nxp/pxp/lv_gpu_nxp_pxp_osa.o ./grapics/src/draw/nxp/pxp/lv_gpu_nxp_pxp_osa.su

.PHONY: clean-grapics-2f-src-2f-draw-2f-nxp-2f-pxp

