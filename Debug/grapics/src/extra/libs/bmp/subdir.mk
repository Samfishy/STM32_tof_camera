################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/src/extra/libs/bmp/lv_bmp.c 

OBJS += \
./grapics/src/extra/libs/bmp/lv_bmp.o 

C_DEPS += \
./grapics/src/extra/libs/bmp/lv_bmp.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/src/extra/libs/bmp/%.o grapics/src/extra/libs/bmp/%.su grapics/src/extra/libs/bmp/%.cyclo: ../grapics/src/extra/libs/bmp/%.c grapics/src/extra/libs/bmp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-src-2f-extra-2f-libs-2f-bmp

clean-grapics-2f-src-2f-extra-2f-libs-2f-bmp:
	-$(RM) ./grapics/src/extra/libs/bmp/lv_bmp.cyclo ./grapics/src/extra/libs/bmp/lv_bmp.d ./grapics/src/extra/libs/bmp/lv_bmp.o ./grapics/src/extra/libs/bmp/lv_bmp.su

.PHONY: clean-grapics-2f-src-2f-extra-2f-libs-2f-bmp

