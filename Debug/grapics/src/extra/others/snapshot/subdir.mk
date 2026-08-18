################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/src/extra/others/snapshot/lv_snapshot.c 

OBJS += \
./grapics/src/extra/others/snapshot/lv_snapshot.o 

C_DEPS += \
./grapics/src/extra/others/snapshot/lv_snapshot.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/src/extra/others/snapshot/%.o grapics/src/extra/others/snapshot/%.su grapics/src/extra/others/snapshot/%.cyclo: ../grapics/src/extra/others/snapshot/%.c grapics/src/extra/others/snapshot/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-src-2f-extra-2f-others-2f-snapshot

clean-grapics-2f-src-2f-extra-2f-others-2f-snapshot:
	-$(RM) ./grapics/src/extra/others/snapshot/lv_snapshot.cyclo ./grapics/src/extra/others/snapshot/lv_snapshot.d ./grapics/src/extra/others/snapshot/lv_snapshot.o ./grapics/src/extra/others/snapshot/lv_snapshot.su

.PHONY: clean-grapics-2f-src-2f-extra-2f-others-2f-snapshot

