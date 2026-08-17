################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/demos/benchmark/assets/img_benchmark_cogwheel_alpha16.c \
../grapics/demos/benchmark/assets/img_benchmark_cogwheel_argb.c \
../grapics/demos/benchmark/assets/img_benchmark_cogwheel_chroma_keyed.c \
../grapics/demos/benchmark/assets/img_benchmark_cogwheel_indexed16.c \
../grapics/demos/benchmark/assets/img_benchmark_cogwheel_rgb.c \
../grapics/demos/benchmark/assets/img_benchmark_cogwheel_rgb565a8.c \
../grapics/demos/benchmark/assets/lv_font_bechmark_montserrat_12_compr_az.c.c \
../grapics/demos/benchmark/assets/lv_font_bechmark_montserrat_16_compr_az.c.c \
../grapics/demos/benchmark/assets/lv_font_bechmark_montserrat_28_compr_az.c.c 

OBJS += \
./grapics/demos/benchmark/assets/img_benchmark_cogwheel_alpha16.o \
./grapics/demos/benchmark/assets/img_benchmark_cogwheel_argb.o \
./grapics/demos/benchmark/assets/img_benchmark_cogwheel_chroma_keyed.o \
./grapics/demos/benchmark/assets/img_benchmark_cogwheel_indexed16.o \
./grapics/demos/benchmark/assets/img_benchmark_cogwheel_rgb.o \
./grapics/demos/benchmark/assets/img_benchmark_cogwheel_rgb565a8.o \
./grapics/demos/benchmark/assets/lv_font_bechmark_montserrat_12_compr_az.c.o \
./grapics/demos/benchmark/assets/lv_font_bechmark_montserrat_16_compr_az.c.o \
./grapics/demos/benchmark/assets/lv_font_bechmark_montserrat_28_compr_az.c.o 

C_DEPS += \
./grapics/demos/benchmark/assets/img_benchmark_cogwheel_alpha16.d \
./grapics/demos/benchmark/assets/img_benchmark_cogwheel_argb.d \
./grapics/demos/benchmark/assets/img_benchmark_cogwheel_chroma_keyed.d \
./grapics/demos/benchmark/assets/img_benchmark_cogwheel_indexed16.d \
./grapics/demos/benchmark/assets/img_benchmark_cogwheel_rgb.d \
./grapics/demos/benchmark/assets/img_benchmark_cogwheel_rgb565a8.d \
./grapics/demos/benchmark/assets/lv_font_bechmark_montserrat_12_compr_az.c.d \
./grapics/demos/benchmark/assets/lv_font_bechmark_montserrat_16_compr_az.c.d \
./grapics/demos/benchmark/assets/lv_font_bechmark_montserrat_28_compr_az.c.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/demos/benchmark/assets/%.o grapics/demos/benchmark/assets/%.su grapics/demos/benchmark/assets/%.cyclo: ../grapics/demos/benchmark/assets/%.c grapics/demos/benchmark/assets/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-demos-2f-benchmark-2f-assets

clean-grapics-2f-demos-2f-benchmark-2f-assets:
	-$(RM) ./grapics/demos/benchmark/assets/img_benchmark_cogwheel_alpha16.cyclo ./grapics/demos/benchmark/assets/img_benchmark_cogwheel_alpha16.d ./grapics/demos/benchmark/assets/img_benchmark_cogwheel_alpha16.o ./grapics/demos/benchmark/assets/img_benchmark_cogwheel_alpha16.su ./grapics/demos/benchmark/assets/img_benchmark_cogwheel_argb.cyclo ./grapics/demos/benchmark/assets/img_benchmark_cogwheel_argb.d ./grapics/demos/benchmark/assets/img_benchmark_cogwheel_argb.o ./grapics/demos/benchmark/assets/img_benchmark_cogwheel_argb.su ./grapics/demos/benchmark/assets/img_benchmark_cogwheel_chroma_keyed.cyclo ./grapics/demos/benchmark/assets/img_benchmark_cogwheel_chroma_keyed.d ./grapics/demos/benchmark/assets/img_benchmark_cogwheel_chroma_keyed.o ./grapics/demos/benchmark/assets/img_benchmark_cogwheel_chroma_keyed.su ./grapics/demos/benchmark/assets/img_benchmark_cogwheel_indexed16.cyclo ./grapics/demos/benchmark/assets/img_benchmark_cogwheel_indexed16.d ./grapics/demos/benchmark/assets/img_benchmark_cogwheel_indexed16.o ./grapics/demos/benchmark/assets/img_benchmark_cogwheel_indexed16.su ./grapics/demos/benchmark/assets/img_benchmark_cogwheel_rgb.cyclo ./grapics/demos/benchmark/assets/img_benchmark_cogwheel_rgb.d ./grapics/demos/benchmark/assets/img_benchmark_cogwheel_rgb.o ./grapics/demos/benchmark/assets/img_benchmark_cogwheel_rgb.su ./grapics/demos/benchmark/assets/img_benchmark_cogwheel_rgb565a8.cyclo ./grapics/demos/benchmark/assets/img_benchmark_cogwheel_rgb565a8.d ./grapics/demos/benchmark/assets/img_benchmark_cogwheel_rgb565a8.o ./grapics/demos/benchmark/assets/img_benchmark_cogwheel_rgb565a8.su ./grapics/demos/benchmark/assets/lv_font_bechmark_montserrat_12_compr_az.c.cyclo ./grapics/demos/benchmark/assets/lv_font_bechmark_montserrat_12_compr_az.c.d ./grapics/demos/benchmark/assets/lv_font_bechmark_montserrat_12_compr_az.c.o ./grapics/demos/benchmark/assets/lv_font_bechmark_montserrat_12_compr_az.c.su ./grapics/demos/benchmark/assets/lv_font_bechmark_montserrat_16_compr_az.c.cyclo ./grapics/demos/benchmark/assets/lv_font_bechmark_montserrat_16_compr_az.c.d ./grapics/demos/benchmark/assets/lv_font_bechmark_montserrat_16_compr_az.c.o ./grapics/demos/benchmark/assets/lv_font_bechmark_montserrat_16_compr_az.c.su ./grapics/demos/benchmark/assets/lv_font_bechmark_montserrat_28_compr_az.c.cyclo ./grapics/demos/benchmark/assets/lv_font_bechmark_montserrat_28_compr_az.c.d ./grapics/demos/benchmark/assets/lv_font_bechmark_montserrat_28_compr_az.c.o ./grapics/demos/benchmark/assets/lv_font_bechmark_montserrat_28_compr_az.c.su

.PHONY: clean-grapics-2f-demos-2f-benchmark-2f-assets

