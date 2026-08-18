################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/tests/src/test_fonts/font_1.c \
../grapics/tests/src/test_fonts/font_2.c \
../grapics/tests/src/test_fonts/font_3.c \
../grapics/tests/src/test_fonts/ubuntu_font.c 

OBJS += \
./grapics/tests/src/test_fonts/font_1.o \
./grapics/tests/src/test_fonts/font_2.o \
./grapics/tests/src/test_fonts/font_3.o \
./grapics/tests/src/test_fonts/ubuntu_font.o 

C_DEPS += \
./grapics/tests/src/test_fonts/font_1.d \
./grapics/tests/src/test_fonts/font_2.d \
./grapics/tests/src/test_fonts/font_3.d \
./grapics/tests/src/test_fonts/ubuntu_font.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/tests/src/test_fonts/%.o grapics/tests/src/test_fonts/%.su grapics/tests/src/test_fonts/%.cyclo: ../grapics/tests/src/test_fonts/%.c grapics/tests/src/test_fonts/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-tests-2f-src-2f-test_fonts

clean-grapics-2f-tests-2f-src-2f-test_fonts:
	-$(RM) ./grapics/tests/src/test_fonts/font_1.cyclo ./grapics/tests/src/test_fonts/font_1.d ./grapics/tests/src/test_fonts/font_1.o ./grapics/tests/src/test_fonts/font_1.su ./grapics/tests/src/test_fonts/font_2.cyclo ./grapics/tests/src/test_fonts/font_2.d ./grapics/tests/src/test_fonts/font_2.o ./grapics/tests/src/test_fonts/font_2.su ./grapics/tests/src/test_fonts/font_3.cyclo ./grapics/tests/src/test_fonts/font_3.d ./grapics/tests/src/test_fonts/font_3.o ./grapics/tests/src/test_fonts/font_3.su ./grapics/tests/src/test_fonts/ubuntu_font.cyclo ./grapics/tests/src/test_fonts/ubuntu_font.d ./grapics/tests/src/test_fonts/ubuntu_font.o ./grapics/tests/src/test_fonts/ubuntu_font.su

.PHONY: clean-grapics-2f-tests-2f-src-2f-test_fonts

