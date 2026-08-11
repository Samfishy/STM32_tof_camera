################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/src/extra/libs/qrcode/lv_qrcode.c \
../grapics/src/extra/libs/qrcode/qrcodegen.c 

OBJS += \
./grapics/src/extra/libs/qrcode/lv_qrcode.o \
./grapics/src/extra/libs/qrcode/qrcodegen.o 

C_DEPS += \
./grapics/src/extra/libs/qrcode/lv_qrcode.d \
./grapics/src/extra/libs/qrcode/qrcodegen.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/src/extra/libs/qrcode/%.o grapics/src/extra/libs/qrcode/%.su grapics/src/extra/libs/qrcode/%.cyclo: ../grapics/src/extra/libs/qrcode/%.c grapics/src/extra/libs/qrcode/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-src-2f-extra-2f-libs-2f-qrcode

clean-grapics-2f-src-2f-extra-2f-libs-2f-qrcode:
	-$(RM) ./grapics/src/extra/libs/qrcode/lv_qrcode.cyclo ./grapics/src/extra/libs/qrcode/lv_qrcode.d ./grapics/src/extra/libs/qrcode/lv_qrcode.o ./grapics/src/extra/libs/qrcode/lv_qrcode.su ./grapics/src/extra/libs/qrcode/qrcodegen.cyclo ./grapics/src/extra/libs/qrcode/qrcodegen.d ./grapics/src/extra/libs/qrcode/qrcodegen.o ./grapics/src/extra/libs/qrcode/qrcodegen.su

.PHONY: clean-grapics-2f-src-2f-extra-2f-libs-2f-qrcode

