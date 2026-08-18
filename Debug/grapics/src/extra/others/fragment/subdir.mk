################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/src/extra/others/fragment/lv_fragment.c \
../grapics/src/extra/others/fragment/lv_fragment_manager.c 

OBJS += \
./grapics/src/extra/others/fragment/lv_fragment.o \
./grapics/src/extra/others/fragment/lv_fragment_manager.o 

C_DEPS += \
./grapics/src/extra/others/fragment/lv_fragment.d \
./grapics/src/extra/others/fragment/lv_fragment_manager.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/src/extra/others/fragment/%.o grapics/src/extra/others/fragment/%.su grapics/src/extra/others/fragment/%.cyclo: ../grapics/src/extra/others/fragment/%.c grapics/src/extra/others/fragment/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-src-2f-extra-2f-others-2f-fragment

clean-grapics-2f-src-2f-extra-2f-others-2f-fragment:
	-$(RM) ./grapics/src/extra/others/fragment/lv_fragment.cyclo ./grapics/src/extra/others/fragment/lv_fragment.d ./grapics/src/extra/others/fragment/lv_fragment.o ./grapics/src/extra/others/fragment/lv_fragment.su ./grapics/src/extra/others/fragment/lv_fragment_manager.cyclo ./grapics/src/extra/others/fragment/lv_fragment_manager.d ./grapics/src/extra/others/fragment/lv_fragment_manager.o ./grapics/src/extra/others/fragment/lv_fragment_manager.su

.PHONY: clean-grapics-2f-src-2f-extra-2f-others-2f-fragment

