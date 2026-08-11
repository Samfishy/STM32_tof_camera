################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/examples/libs/gif/img_bulb_gif.c \
../grapics/examples/libs/gif/lv_example_gif_1.c 

OBJS += \
./grapics/examples/libs/gif/img_bulb_gif.o \
./grapics/examples/libs/gif/lv_example_gif_1.o 

C_DEPS += \
./grapics/examples/libs/gif/img_bulb_gif.d \
./grapics/examples/libs/gif/lv_example_gif_1.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/examples/libs/gif/%.o grapics/examples/libs/gif/%.su grapics/examples/libs/gif/%.cyclo: ../grapics/examples/libs/gif/%.c grapics/examples/libs/gif/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-examples-2f-libs-2f-gif

clean-grapics-2f-examples-2f-libs-2f-gif:
	-$(RM) ./grapics/examples/libs/gif/img_bulb_gif.cyclo ./grapics/examples/libs/gif/img_bulb_gif.d ./grapics/examples/libs/gif/img_bulb_gif.o ./grapics/examples/libs/gif/img_bulb_gif.su ./grapics/examples/libs/gif/lv_example_gif_1.cyclo ./grapics/examples/libs/gif/lv_example_gif_1.d ./grapics/examples/libs/gif/lv_example_gif_1.o ./grapics/examples/libs/gif/lv_example_gif_1.su

.PHONY: clean-grapics-2f-examples-2f-libs-2f-gif

