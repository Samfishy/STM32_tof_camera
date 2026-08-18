################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/demos/widgets/assets/img_clothes.c \
../grapics/demos/widgets/assets/img_demo_widgets_avatar.c \
../grapics/demos/widgets/assets/img_lvgl_logo.c 

OBJS += \
./grapics/demos/widgets/assets/img_clothes.o \
./grapics/demos/widgets/assets/img_demo_widgets_avatar.o \
./grapics/demos/widgets/assets/img_lvgl_logo.o 

C_DEPS += \
./grapics/demos/widgets/assets/img_clothes.d \
./grapics/demos/widgets/assets/img_demo_widgets_avatar.d \
./grapics/demos/widgets/assets/img_lvgl_logo.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/demos/widgets/assets/%.o grapics/demos/widgets/assets/%.su grapics/demos/widgets/assets/%.cyclo: ../grapics/demos/widgets/assets/%.c grapics/demos/widgets/assets/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-demos-2f-widgets-2f-assets

clean-grapics-2f-demos-2f-widgets-2f-assets:
	-$(RM) ./grapics/demos/widgets/assets/img_clothes.cyclo ./grapics/demos/widgets/assets/img_clothes.d ./grapics/demos/widgets/assets/img_clothes.o ./grapics/demos/widgets/assets/img_clothes.su ./grapics/demos/widgets/assets/img_demo_widgets_avatar.cyclo ./grapics/demos/widgets/assets/img_demo_widgets_avatar.d ./grapics/demos/widgets/assets/img_demo_widgets_avatar.o ./grapics/demos/widgets/assets/img_demo_widgets_avatar.su ./grapics/demos/widgets/assets/img_lvgl_logo.cyclo ./grapics/demos/widgets/assets/img_lvgl_logo.d ./grapics/demos/widgets/assets/img_lvgl_logo.o ./grapics/demos/widgets/assets/img_lvgl_logo.su

.PHONY: clean-grapics-2f-demos-2f-widgets-2f-assets

