################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/examples/others/gridnav/lv_example_gridnav_1.c \
../grapics/examples/others/gridnav/lv_example_gridnav_2.c \
../grapics/examples/others/gridnav/lv_example_gridnav_3.c \
../grapics/examples/others/gridnav/lv_example_gridnav_4.c 

OBJS += \
./grapics/examples/others/gridnav/lv_example_gridnav_1.o \
./grapics/examples/others/gridnav/lv_example_gridnav_2.o \
./grapics/examples/others/gridnav/lv_example_gridnav_3.o \
./grapics/examples/others/gridnav/lv_example_gridnav_4.o 

C_DEPS += \
./grapics/examples/others/gridnav/lv_example_gridnav_1.d \
./grapics/examples/others/gridnav/lv_example_gridnav_2.d \
./grapics/examples/others/gridnav/lv_example_gridnav_3.d \
./grapics/examples/others/gridnav/lv_example_gridnav_4.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/examples/others/gridnav/%.o grapics/examples/others/gridnav/%.su grapics/examples/others/gridnav/%.cyclo: ../grapics/examples/others/gridnav/%.c grapics/examples/others/gridnav/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-examples-2f-others-2f-gridnav

clean-grapics-2f-examples-2f-others-2f-gridnav:
	-$(RM) ./grapics/examples/others/gridnav/lv_example_gridnav_1.cyclo ./grapics/examples/others/gridnav/lv_example_gridnav_1.d ./grapics/examples/others/gridnav/lv_example_gridnav_1.o ./grapics/examples/others/gridnav/lv_example_gridnav_1.su ./grapics/examples/others/gridnav/lv_example_gridnav_2.cyclo ./grapics/examples/others/gridnav/lv_example_gridnav_2.d ./grapics/examples/others/gridnav/lv_example_gridnav_2.o ./grapics/examples/others/gridnav/lv_example_gridnav_2.su ./grapics/examples/others/gridnav/lv_example_gridnav_3.cyclo ./grapics/examples/others/gridnav/lv_example_gridnav_3.d ./grapics/examples/others/gridnav/lv_example_gridnav_3.o ./grapics/examples/others/gridnav/lv_example_gridnav_3.su ./grapics/examples/others/gridnav/lv_example_gridnav_4.cyclo ./grapics/examples/others/gridnav/lv_example_gridnav_4.d ./grapics/examples/others/gridnav/lv_example_gridnav_4.o ./grapics/examples/others/gridnav/lv_example_gridnav_4.su

.PHONY: clean-grapics-2f-examples-2f-others-2f-gridnav

