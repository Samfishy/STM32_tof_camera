################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/examples/libs/ffmpeg/lv_example_ffmpeg_1.c \
../grapics/examples/libs/ffmpeg/lv_example_ffmpeg_2.c 

OBJS += \
./grapics/examples/libs/ffmpeg/lv_example_ffmpeg_1.o \
./grapics/examples/libs/ffmpeg/lv_example_ffmpeg_2.o 

C_DEPS += \
./grapics/examples/libs/ffmpeg/lv_example_ffmpeg_1.d \
./grapics/examples/libs/ffmpeg/lv_example_ffmpeg_2.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/examples/libs/ffmpeg/%.o grapics/examples/libs/ffmpeg/%.su grapics/examples/libs/ffmpeg/%.cyclo: ../grapics/examples/libs/ffmpeg/%.c grapics/examples/libs/ffmpeg/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-examples-2f-libs-2f-ffmpeg

clean-grapics-2f-examples-2f-libs-2f-ffmpeg:
	-$(RM) ./grapics/examples/libs/ffmpeg/lv_example_ffmpeg_1.cyclo ./grapics/examples/libs/ffmpeg/lv_example_ffmpeg_1.d ./grapics/examples/libs/ffmpeg/lv_example_ffmpeg_1.o ./grapics/examples/libs/ffmpeg/lv_example_ffmpeg_1.su ./grapics/examples/libs/ffmpeg/lv_example_ffmpeg_2.cyclo ./grapics/examples/libs/ffmpeg/lv_example_ffmpeg_2.d ./grapics/examples/libs/ffmpeg/lv_example_ffmpeg_2.o ./grapics/examples/libs/ffmpeg/lv_example_ffmpeg_2.su

.PHONY: clean-grapics-2f-examples-2f-libs-2f-ffmpeg

