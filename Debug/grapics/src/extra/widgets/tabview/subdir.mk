################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/src/extra/widgets/tabview/lv_tabview.c 

OBJS += \
./grapics/src/extra/widgets/tabview/lv_tabview.o 

C_DEPS += \
./grapics/src/extra/widgets/tabview/lv_tabview.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/src/extra/widgets/tabview/%.o grapics/src/extra/widgets/tabview/%.su grapics/src/extra/widgets/tabview/%.cyclo: ../grapics/src/extra/widgets/tabview/%.c grapics/src/extra/widgets/tabview/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-src-2f-extra-2f-widgets-2f-tabview

clean-grapics-2f-src-2f-extra-2f-widgets-2f-tabview:
	-$(RM) ./grapics/src/extra/widgets/tabview/lv_tabview.cyclo ./grapics/src/extra/widgets/tabview/lv_tabview.d ./grapics/src/extra/widgets/tabview/lv_tabview.o ./grapics/src/extra/widgets/tabview/lv_tabview.su

.PHONY: clean-grapics-2f-src-2f-extra-2f-widgets-2f-tabview

