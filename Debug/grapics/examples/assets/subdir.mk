################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/examples/assets/animimg001.c \
../grapics/examples/assets/animimg002.c \
../grapics/examples/assets/animimg003.c \
../grapics/examples/assets/img_caret_down.c \
../grapics/examples/assets/img_cogwheel_alpha16.c \
../grapics/examples/assets/img_cogwheel_argb.c \
../grapics/examples/assets/img_cogwheel_chroma_keyed.c \
../grapics/examples/assets/img_cogwheel_indexed16.c \
../grapics/examples/assets/img_cogwheel_rgb.c \
../grapics/examples/assets/img_hand.c \
../grapics/examples/assets/img_skew_strip.c \
../grapics/examples/assets/img_star.c \
../grapics/examples/assets/imgbtn_left.c \
../grapics/examples/assets/imgbtn_mid.c \
../grapics/examples/assets/imgbtn_right.c 

OBJS += \
./grapics/examples/assets/animimg001.o \
./grapics/examples/assets/animimg002.o \
./grapics/examples/assets/animimg003.o \
./grapics/examples/assets/img_caret_down.o \
./grapics/examples/assets/img_cogwheel_alpha16.o \
./grapics/examples/assets/img_cogwheel_argb.o \
./grapics/examples/assets/img_cogwheel_chroma_keyed.o \
./grapics/examples/assets/img_cogwheel_indexed16.o \
./grapics/examples/assets/img_cogwheel_rgb.o \
./grapics/examples/assets/img_hand.o \
./grapics/examples/assets/img_skew_strip.o \
./grapics/examples/assets/img_star.o \
./grapics/examples/assets/imgbtn_left.o \
./grapics/examples/assets/imgbtn_mid.o \
./grapics/examples/assets/imgbtn_right.o 

C_DEPS += \
./grapics/examples/assets/animimg001.d \
./grapics/examples/assets/animimg002.d \
./grapics/examples/assets/animimg003.d \
./grapics/examples/assets/img_caret_down.d \
./grapics/examples/assets/img_cogwheel_alpha16.d \
./grapics/examples/assets/img_cogwheel_argb.d \
./grapics/examples/assets/img_cogwheel_chroma_keyed.d \
./grapics/examples/assets/img_cogwheel_indexed16.d \
./grapics/examples/assets/img_cogwheel_rgb.d \
./grapics/examples/assets/img_hand.d \
./grapics/examples/assets/img_skew_strip.d \
./grapics/examples/assets/img_star.d \
./grapics/examples/assets/imgbtn_left.d \
./grapics/examples/assets/imgbtn_mid.d \
./grapics/examples/assets/imgbtn_right.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/examples/assets/%.o grapics/examples/assets/%.su grapics/examples/assets/%.cyclo: ../grapics/examples/assets/%.c grapics/examples/assets/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-examples-2f-assets

clean-grapics-2f-examples-2f-assets:
	-$(RM) ./grapics/examples/assets/animimg001.cyclo ./grapics/examples/assets/animimg001.d ./grapics/examples/assets/animimg001.o ./grapics/examples/assets/animimg001.su ./grapics/examples/assets/animimg002.cyclo ./grapics/examples/assets/animimg002.d ./grapics/examples/assets/animimg002.o ./grapics/examples/assets/animimg002.su ./grapics/examples/assets/animimg003.cyclo ./grapics/examples/assets/animimg003.d ./grapics/examples/assets/animimg003.o ./grapics/examples/assets/animimg003.su ./grapics/examples/assets/img_caret_down.cyclo ./grapics/examples/assets/img_caret_down.d ./grapics/examples/assets/img_caret_down.o ./grapics/examples/assets/img_caret_down.su ./grapics/examples/assets/img_cogwheel_alpha16.cyclo ./grapics/examples/assets/img_cogwheel_alpha16.d ./grapics/examples/assets/img_cogwheel_alpha16.o ./grapics/examples/assets/img_cogwheel_alpha16.su ./grapics/examples/assets/img_cogwheel_argb.cyclo ./grapics/examples/assets/img_cogwheel_argb.d ./grapics/examples/assets/img_cogwheel_argb.o ./grapics/examples/assets/img_cogwheel_argb.su ./grapics/examples/assets/img_cogwheel_chroma_keyed.cyclo ./grapics/examples/assets/img_cogwheel_chroma_keyed.d ./grapics/examples/assets/img_cogwheel_chroma_keyed.o ./grapics/examples/assets/img_cogwheel_chroma_keyed.su ./grapics/examples/assets/img_cogwheel_indexed16.cyclo ./grapics/examples/assets/img_cogwheel_indexed16.d ./grapics/examples/assets/img_cogwheel_indexed16.o ./grapics/examples/assets/img_cogwheel_indexed16.su ./grapics/examples/assets/img_cogwheel_rgb.cyclo ./grapics/examples/assets/img_cogwheel_rgb.d ./grapics/examples/assets/img_cogwheel_rgb.o ./grapics/examples/assets/img_cogwheel_rgb.su ./grapics/examples/assets/img_hand.cyclo ./grapics/examples/assets/img_hand.d ./grapics/examples/assets/img_hand.o ./grapics/examples/assets/img_hand.su ./grapics/examples/assets/img_skew_strip.cyclo ./grapics/examples/assets/img_skew_strip.d ./grapics/examples/assets/img_skew_strip.o ./grapics/examples/assets/img_skew_strip.su ./grapics/examples/assets/img_star.cyclo ./grapics/examples/assets/img_star.d ./grapics/examples/assets/img_star.o ./grapics/examples/assets/img_star.su ./grapics/examples/assets/imgbtn_left.cyclo ./grapics/examples/assets/imgbtn_left.d ./grapics/examples/assets/imgbtn_left.o ./grapics/examples/assets/imgbtn_left.su ./grapics/examples/assets/imgbtn_mid.cyclo ./grapics/examples/assets/imgbtn_mid.d ./grapics/examples/assets/imgbtn_mid.o ./grapics/examples/assets/imgbtn_mid.su ./grapics/examples/assets/imgbtn_right.cyclo ./grapics/examples/assets/imgbtn_right.d ./grapics/examples/assets/imgbtn_right.o ./grapics/examples/assets/imgbtn_right.su

.PHONY: clean-grapics-2f-examples-2f-assets

