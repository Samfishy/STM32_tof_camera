################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lvgl/src/libs/gltf/gltf_view/assets/chromatic.c \
../Core/lvgl/src/libs/gltf/gltf_view/assets/lv_gltf_view_shader.c 

OBJS += \
./Core/lvgl/src/libs/gltf/gltf_view/assets/chromatic.o \
./Core/lvgl/src/libs/gltf/gltf_view/assets/lv_gltf_view_shader.o 

C_DEPS += \
./Core/lvgl/src/libs/gltf/gltf_view/assets/chromatic.d \
./Core/lvgl/src/libs/gltf/gltf_view/assets/lv_gltf_view_shader.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lvgl/src/libs/gltf/gltf_view/assets/%.o Core/lvgl/src/libs/gltf/gltf_view/assets/%.su Core/lvgl/src/libs/gltf/gltf_view/assets/%.cyclo: ../Core/lvgl/src/libs/gltf/gltf_view/assets/%.c Core/lvgl/src/libs/gltf/gltf_view/assets/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/lvgl" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lvgl-2f-src-2f-libs-2f-gltf-2f-gltf_view-2f-assets

clean-Core-2f-lvgl-2f-src-2f-libs-2f-gltf-2f-gltf_view-2f-assets:
	-$(RM) ./Core/lvgl/src/libs/gltf/gltf_view/assets/chromatic.cyclo ./Core/lvgl/src/libs/gltf/gltf_view/assets/chromatic.d ./Core/lvgl/src/libs/gltf/gltf_view/assets/chromatic.o ./Core/lvgl/src/libs/gltf/gltf_view/assets/chromatic.su ./Core/lvgl/src/libs/gltf/gltf_view/assets/lv_gltf_view_shader.cyclo ./Core/lvgl/src/libs/gltf/gltf_view/assets/lv_gltf_view_shader.d ./Core/lvgl/src/libs/gltf/gltf_view/assets/lv_gltf_view_shader.o ./Core/lvgl/src/libs/gltf/gltf_view/assets/lv_gltf_view_shader.su

.PHONY: clean-Core-2f-lvgl-2f-src-2f-libs-2f-gltf-2f-gltf_view-2f-assets

