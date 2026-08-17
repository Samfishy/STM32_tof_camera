################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lvgl/tests/test_images/stride_align1/RLE/test_A1_RLE_align1.c \
../Core/lvgl/tests/test_images/stride_align1/RLE/test_A2_RLE_align1.c \
../Core/lvgl/tests/test_images/stride_align1/RLE/test_A4_RLE_align1.c \
../Core/lvgl/tests/test_images/stride_align1/RLE/test_A8_RLE_align1.c \
../Core/lvgl/tests/test_images/stride_align1/RLE/test_ARGB8565_RLE_align1.c \
../Core/lvgl/tests/test_images/stride_align1/RLE/test_ARGB8888_PREMULTIPLIED_RLE_align1.c \
../Core/lvgl/tests/test_images/stride_align1/RLE/test_ARGB8888_RLE_align1.c \
../Core/lvgl/tests/test_images/stride_align1/RLE/test_I1_RLE_align1.c \
../Core/lvgl/tests/test_images/stride_align1/RLE/test_I2_RLE_align1.c \
../Core/lvgl/tests/test_images/stride_align1/RLE/test_I4_RLE_align1.c \
../Core/lvgl/tests/test_images/stride_align1/RLE/test_I8_RLE_align1.c \
../Core/lvgl/tests/test_images/stride_align1/RLE/test_L8_RLE_align1.c \
../Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB565A8_RLE_align1.c \
../Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB565_RLE_align1.c \
../Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB565_SWAPPED_RLE_align1.c \
../Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB888_RLE_align1.c \
../Core/lvgl/tests/test_images/stride_align1/RLE/test_XRGB8888_RLE_align1.c 

OBJS += \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_A1_RLE_align1.o \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_A2_RLE_align1.o \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_A4_RLE_align1.o \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_A8_RLE_align1.o \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_ARGB8565_RLE_align1.o \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_ARGB8888_PREMULTIPLIED_RLE_align1.o \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_ARGB8888_RLE_align1.o \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_I1_RLE_align1.o \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_I2_RLE_align1.o \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_I4_RLE_align1.o \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_I8_RLE_align1.o \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_L8_RLE_align1.o \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB565A8_RLE_align1.o \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB565_RLE_align1.o \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB565_SWAPPED_RLE_align1.o \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB888_RLE_align1.o \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_XRGB8888_RLE_align1.o 

C_DEPS += \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_A1_RLE_align1.d \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_A2_RLE_align1.d \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_A4_RLE_align1.d \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_A8_RLE_align1.d \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_ARGB8565_RLE_align1.d \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_ARGB8888_PREMULTIPLIED_RLE_align1.d \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_ARGB8888_RLE_align1.d \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_I1_RLE_align1.d \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_I2_RLE_align1.d \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_I4_RLE_align1.d \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_I8_RLE_align1.d \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_L8_RLE_align1.d \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB565A8_RLE_align1.d \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB565_RLE_align1.d \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB565_SWAPPED_RLE_align1.d \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB888_RLE_align1.d \
./Core/lvgl/tests/test_images/stride_align1/RLE/test_XRGB8888_RLE_align1.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lvgl/tests/test_images/stride_align1/RLE/%.o Core/lvgl/tests/test_images/stride_align1/RLE/%.su Core/lvgl/tests/test_images/stride_align1/RLE/%.cyclo: ../Core/lvgl/tests/test_images/stride_align1/RLE/%.c Core/lvgl/tests/test_images/stride_align1/RLE/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/lvgl" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lvgl-2f-tests-2f-test_images-2f-stride_align1-2f-RLE

clean-Core-2f-lvgl-2f-tests-2f-test_images-2f-stride_align1-2f-RLE:
	-$(RM) ./Core/lvgl/tests/test_images/stride_align1/RLE/test_A1_RLE_align1.cyclo ./Core/lvgl/tests/test_images/stride_align1/RLE/test_A1_RLE_align1.d ./Core/lvgl/tests/test_images/stride_align1/RLE/test_A1_RLE_align1.o ./Core/lvgl/tests/test_images/stride_align1/RLE/test_A1_RLE_align1.su ./Core/lvgl/tests/test_images/stride_align1/RLE/test_A2_RLE_align1.cyclo ./Core/lvgl/tests/test_images/stride_align1/RLE/test_A2_RLE_align1.d ./Core/lvgl/tests/test_images/stride_align1/RLE/test_A2_RLE_align1.o ./Core/lvgl/tests/test_images/stride_align1/RLE/test_A2_RLE_align1.su ./Core/lvgl/tests/test_images/stride_align1/RLE/test_A4_RLE_align1.cyclo ./Core/lvgl/tests/test_images/stride_align1/RLE/test_A4_RLE_align1.d ./Core/lvgl/tests/test_images/stride_align1/RLE/test_A4_RLE_align1.o ./Core/lvgl/tests/test_images/stride_align1/RLE/test_A4_RLE_align1.su ./Core/lvgl/tests/test_images/stride_align1/RLE/test_A8_RLE_align1.cyclo ./Core/lvgl/tests/test_images/stride_align1/RLE/test_A8_RLE_align1.d ./Core/lvgl/tests/test_images/stride_align1/RLE/test_A8_RLE_align1.o ./Core/lvgl/tests/test_images/stride_align1/RLE/test_A8_RLE_align1.su ./Core/lvgl/tests/test_images/stride_align1/RLE/test_ARGB8565_RLE_align1.cyclo ./Core/lvgl/tests/test_images/stride_align1/RLE/test_ARGB8565_RLE_align1.d ./Core/lvgl/tests/test_images/stride_align1/RLE/test_ARGB8565_RLE_align1.o ./Core/lvgl/tests/test_images/stride_align1/RLE/test_ARGB8565_RLE_align1.su ./Core/lvgl/tests/test_images/stride_align1/RLE/test_ARGB8888_PREMULTIPLIED_RLE_align1.cyclo ./Core/lvgl/tests/test_images/stride_align1/RLE/test_ARGB8888_PREMULTIPLIED_RLE_align1.d ./Core/lvgl/tests/test_images/stride_align1/RLE/test_ARGB8888_PREMULTIPLIED_RLE_align1.o ./Core/lvgl/tests/test_images/stride_align1/RLE/test_ARGB8888_PREMULTIPLIED_RLE_align1.su ./Core/lvgl/tests/test_images/stride_align1/RLE/test_ARGB8888_RLE_align1.cyclo ./Core/lvgl/tests/test_images/stride_align1/RLE/test_ARGB8888_RLE_align1.d ./Core/lvgl/tests/test_images/stride_align1/RLE/test_ARGB8888_RLE_align1.o ./Core/lvgl/tests/test_images/stride_align1/RLE/test_ARGB8888_RLE_align1.su ./Core/lvgl/tests/test_images/stride_align1/RLE/test_I1_RLE_align1.cyclo ./Core/lvgl/tests/test_images/stride_align1/RLE/test_I1_RLE_align1.d ./Core/lvgl/tests/test_images/stride_align1/RLE/test_I1_RLE_align1.o ./Core/lvgl/tests/test_images/stride_align1/RLE/test_I1_RLE_align1.su ./Core/lvgl/tests/test_images/stride_align1/RLE/test_I2_RLE_align1.cyclo ./Core/lvgl/tests/test_images/stride_align1/RLE/test_I2_RLE_align1.d ./Core/lvgl/tests/test_images/stride_align1/RLE/test_I2_RLE_align1.o ./Core/lvgl/tests/test_images/stride_align1/RLE/test_I2_RLE_align1.su ./Core/lvgl/tests/test_images/stride_align1/RLE/test_I4_RLE_align1.cyclo ./Core/lvgl/tests/test_images/stride_align1/RLE/test_I4_RLE_align1.d ./Core/lvgl/tests/test_images/stride_align1/RLE/test_I4_RLE_align1.o ./Core/lvgl/tests/test_images/stride_align1/RLE/test_I4_RLE_align1.su ./Core/lvgl/tests/test_images/stride_align1/RLE/test_I8_RLE_align1.cyclo ./Core/lvgl/tests/test_images/stride_align1/RLE/test_I8_RLE_align1.d ./Core/lvgl/tests/test_images/stride_align1/RLE/test_I8_RLE_align1.o ./Core/lvgl/tests/test_images/stride_align1/RLE/test_I8_RLE_align1.su ./Core/lvgl/tests/test_images/stride_align1/RLE/test_L8_RLE_align1.cyclo ./Core/lvgl/tests/test_images/stride_align1/RLE/test_L8_RLE_align1.d ./Core/lvgl/tests/test_images/stride_align1/RLE/test_L8_RLE_align1.o ./Core/lvgl/tests/test_images/stride_align1/RLE/test_L8_RLE_align1.su ./Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB565A8_RLE_align1.cyclo ./Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB565A8_RLE_align1.d ./Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB565A8_RLE_align1.o ./Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB565A8_RLE_align1.su ./Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB565_RLE_align1.cyclo ./Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB565_RLE_align1.d ./Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB565_RLE_align1.o ./Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB565_RLE_align1.su ./Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB565_SWAPPED_RLE_align1.cyclo ./Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB565_SWAPPED_RLE_align1.d ./Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB565_SWAPPED_RLE_align1.o ./Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB565_SWAPPED_RLE_align1.su ./Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB888_RLE_align1.cyclo ./Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB888_RLE_align1.d ./Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB888_RLE_align1.o ./Core/lvgl/tests/test_images/stride_align1/RLE/test_RGB888_RLE_align1.su ./Core/lvgl/tests/test_images/stride_align1/RLE/test_XRGB8888_RLE_align1.cyclo ./Core/lvgl/tests/test_images/stride_align1/RLE/test_XRGB8888_RLE_align1.d ./Core/lvgl/tests/test_images/stride_align1/RLE/test_XRGB8888_RLE_align1.o ./Core/lvgl/tests/test_images/stride_align1/RLE/test_XRGB8888_RLE_align1.su

.PHONY: clean-Core-2f-lvgl-2f-tests-2f-test_images-2f-stride_align1-2f-RLE

