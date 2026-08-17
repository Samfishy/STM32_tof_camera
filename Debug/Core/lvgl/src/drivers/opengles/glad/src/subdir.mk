################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lvgl/src/drivers/opengles/glad/src/egl.c \
../Core/lvgl/src/drivers/opengles/glad/src/gl.c \
../Core/lvgl/src/drivers/opengles/glad/src/gles2.c 

OBJS += \
./Core/lvgl/src/drivers/opengles/glad/src/egl.o \
./Core/lvgl/src/drivers/opengles/glad/src/gl.o \
./Core/lvgl/src/drivers/opengles/glad/src/gles2.o 

C_DEPS += \
./Core/lvgl/src/drivers/opengles/glad/src/egl.d \
./Core/lvgl/src/drivers/opengles/glad/src/gl.d \
./Core/lvgl/src/drivers/opengles/glad/src/gles2.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lvgl/src/drivers/opengles/glad/src/%.o Core/lvgl/src/drivers/opengles/glad/src/%.su Core/lvgl/src/drivers/opengles/glad/src/%.cyclo: ../Core/lvgl/src/drivers/opengles/glad/src/%.c Core/lvgl/src/drivers/opengles/glad/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/lvgl" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lvgl-2f-src-2f-drivers-2f-opengles-2f-glad-2f-src

clean-Core-2f-lvgl-2f-src-2f-drivers-2f-opengles-2f-glad-2f-src:
	-$(RM) ./Core/lvgl/src/drivers/opengles/glad/src/egl.cyclo ./Core/lvgl/src/drivers/opengles/glad/src/egl.d ./Core/lvgl/src/drivers/opengles/glad/src/egl.o ./Core/lvgl/src/drivers/opengles/glad/src/egl.su ./Core/lvgl/src/drivers/opengles/glad/src/gl.cyclo ./Core/lvgl/src/drivers/opengles/glad/src/gl.d ./Core/lvgl/src/drivers/opengles/glad/src/gl.o ./Core/lvgl/src/drivers/opengles/glad/src/gl.su ./Core/lvgl/src/drivers/opengles/glad/src/gles2.cyclo ./Core/lvgl/src/drivers/opengles/glad/src/gles2.d ./Core/lvgl/src/drivers/opengles/glad/src/gles2.o ./Core/lvgl/src/drivers/opengles/glad/src/gles2.su

.PHONY: clean-Core-2f-lvgl-2f-src-2f-drivers-2f-opengles-2f-glad-2f-src

