################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/examples/layouts/grid/lv_example_grid_1.c \
../grapics/examples/layouts/grid/lv_example_grid_2.c \
../grapics/examples/layouts/grid/lv_example_grid_3.c \
../grapics/examples/layouts/grid/lv_example_grid_4.c \
../grapics/examples/layouts/grid/lv_example_grid_5.c \
../grapics/examples/layouts/grid/lv_example_grid_6.c 

OBJS += \
./grapics/examples/layouts/grid/lv_example_grid_1.o \
./grapics/examples/layouts/grid/lv_example_grid_2.o \
./grapics/examples/layouts/grid/lv_example_grid_3.o \
./grapics/examples/layouts/grid/lv_example_grid_4.o \
./grapics/examples/layouts/grid/lv_example_grid_5.o \
./grapics/examples/layouts/grid/lv_example_grid_6.o 

C_DEPS += \
./grapics/examples/layouts/grid/lv_example_grid_1.d \
./grapics/examples/layouts/grid/lv_example_grid_2.d \
./grapics/examples/layouts/grid/lv_example_grid_3.d \
./grapics/examples/layouts/grid/lv_example_grid_4.d \
./grapics/examples/layouts/grid/lv_example_grid_5.d \
./grapics/examples/layouts/grid/lv_example_grid_6.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/examples/layouts/grid/%.o grapics/examples/layouts/grid/%.su grapics/examples/layouts/grid/%.cyclo: ../grapics/examples/layouts/grid/%.c grapics/examples/layouts/grid/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-examples-2f-layouts-2f-grid

clean-grapics-2f-examples-2f-layouts-2f-grid:
	-$(RM) ./grapics/examples/layouts/grid/lv_example_grid_1.cyclo ./grapics/examples/layouts/grid/lv_example_grid_1.d ./grapics/examples/layouts/grid/lv_example_grid_1.o ./grapics/examples/layouts/grid/lv_example_grid_1.su ./grapics/examples/layouts/grid/lv_example_grid_2.cyclo ./grapics/examples/layouts/grid/lv_example_grid_2.d ./grapics/examples/layouts/grid/lv_example_grid_2.o ./grapics/examples/layouts/grid/lv_example_grid_2.su ./grapics/examples/layouts/grid/lv_example_grid_3.cyclo ./grapics/examples/layouts/grid/lv_example_grid_3.d ./grapics/examples/layouts/grid/lv_example_grid_3.o ./grapics/examples/layouts/grid/lv_example_grid_3.su ./grapics/examples/layouts/grid/lv_example_grid_4.cyclo ./grapics/examples/layouts/grid/lv_example_grid_4.d ./grapics/examples/layouts/grid/lv_example_grid_4.o ./grapics/examples/layouts/grid/lv_example_grid_4.su ./grapics/examples/layouts/grid/lv_example_grid_5.cyclo ./grapics/examples/layouts/grid/lv_example_grid_5.d ./grapics/examples/layouts/grid/lv_example_grid_5.o ./grapics/examples/layouts/grid/lv_example_grid_5.su ./grapics/examples/layouts/grid/lv_example_grid_6.cyclo ./grapics/examples/layouts/grid/lv_example_grid_6.d ./grapics/examples/layouts/grid/lv_example_grid_6.o ./grapics/examples/layouts/grid/lv_example_grid_6.su

.PHONY: clean-grapics-2f-examples-2f-layouts-2f-grid

