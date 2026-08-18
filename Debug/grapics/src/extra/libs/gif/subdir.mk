################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/src/extra/libs/gif/gifdec.c \
../grapics/src/extra/libs/gif/lv_gif.c 

OBJS += \
./grapics/src/extra/libs/gif/gifdec.o \
./grapics/src/extra/libs/gif/lv_gif.o 

C_DEPS += \
./grapics/src/extra/libs/gif/gifdec.d \
./grapics/src/extra/libs/gif/lv_gif.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/src/extra/libs/gif/%.o grapics/src/extra/libs/gif/%.su grapics/src/extra/libs/gif/%.cyclo: ../grapics/src/extra/libs/gif/%.c grapics/src/extra/libs/gif/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-src-2f-extra-2f-libs-2f-gif

clean-grapics-2f-src-2f-extra-2f-libs-2f-gif:
	-$(RM) ./grapics/src/extra/libs/gif/gifdec.cyclo ./grapics/src/extra/libs/gif/gifdec.d ./grapics/src/extra/libs/gif/gifdec.o ./grapics/src/extra/libs/gif/gifdec.su ./grapics/src/extra/libs/gif/lv_gif.cyclo ./grapics/src/extra/libs/gif/lv_gif.d ./grapics/src/extra/libs/gif/lv_gif.o ./grapics/src/extra/libs/gif/lv_gif.su

.PHONY: clean-grapics-2f-src-2f-extra-2f-libs-2f-gif

