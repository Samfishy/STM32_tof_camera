################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/examples/assets/emoji/img_emoji_F617.c 

OBJS += \
./grapics/examples/assets/emoji/img_emoji_F617.o 

C_DEPS += \
./grapics/examples/assets/emoji/img_emoji_F617.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/examples/assets/emoji/%.o grapics/examples/assets/emoji/%.su grapics/examples/assets/emoji/%.cyclo: ../grapics/examples/assets/emoji/%.c grapics/examples/assets/emoji/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-examples-2f-assets-2f-emoji

clean-grapics-2f-examples-2f-assets-2f-emoji:
	-$(RM) ./grapics/examples/assets/emoji/img_emoji_F617.cyclo ./grapics/examples/assets/emoji/img_emoji_F617.d ./grapics/examples/assets/emoji/img_emoji_F617.o ./grapics/examples/assets/emoji/img_emoji_F617.su

.PHONY: clean-grapics-2f-examples-2f-assets-2f-emoji

