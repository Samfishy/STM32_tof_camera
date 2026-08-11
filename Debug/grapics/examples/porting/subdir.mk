################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/examples/porting/lv_port_disp_template.c \
../grapics/examples/porting/lv_port_fs_template.c \
../grapics/examples/porting/lv_port_indev_template.c 

OBJS += \
./grapics/examples/porting/lv_port_disp_template.o \
./grapics/examples/porting/lv_port_fs_template.o \
./grapics/examples/porting/lv_port_indev_template.o 

C_DEPS += \
./grapics/examples/porting/lv_port_disp_template.d \
./grapics/examples/porting/lv_port_fs_template.d \
./grapics/examples/porting/lv_port_indev_template.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/examples/porting/%.o grapics/examples/porting/%.su grapics/examples/porting/%.cyclo: ../grapics/examples/porting/%.c grapics/examples/porting/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-examples-2f-porting

clean-grapics-2f-examples-2f-porting:
	-$(RM) ./grapics/examples/porting/lv_port_disp_template.cyclo ./grapics/examples/porting/lv_port_disp_template.d ./grapics/examples/porting/lv_port_disp_template.o ./grapics/examples/porting/lv_port_disp_template.su ./grapics/examples/porting/lv_port_fs_template.cyclo ./grapics/examples/porting/lv_port_fs_template.d ./grapics/examples/porting/lv_port_fs_template.o ./grapics/examples/porting/lv_port_fs_template.su ./grapics/examples/porting/lv_port_indev_template.cyclo ./grapics/examples/porting/lv_port_indev_template.d ./grapics/examples/porting/lv_port_indev_template.o ./grapics/examples/porting/lv_port_indev_template.su

.PHONY: clean-grapics-2f-examples-2f-porting

