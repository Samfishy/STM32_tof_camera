################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/src/widgets/lv_arc.c \
../grapics/src/widgets/lv_bar.c \
../grapics/src/widgets/lv_btn.c \
../grapics/src/widgets/lv_btnmatrix.c \
../grapics/src/widgets/lv_canvas.c \
../grapics/src/widgets/lv_checkbox.c \
../grapics/src/widgets/lv_dropdown.c \
../grapics/src/widgets/lv_img.c \
../grapics/src/widgets/lv_label.c \
../grapics/src/widgets/lv_line.c \
../grapics/src/widgets/lv_objx_templ.c \
../grapics/src/widgets/lv_roller.c \
../grapics/src/widgets/lv_slider.c \
../grapics/src/widgets/lv_switch.c \
../grapics/src/widgets/lv_table.c \
../grapics/src/widgets/lv_textarea.c 

OBJS += \
./grapics/src/widgets/lv_arc.o \
./grapics/src/widgets/lv_bar.o \
./grapics/src/widgets/lv_btn.o \
./grapics/src/widgets/lv_btnmatrix.o \
./grapics/src/widgets/lv_canvas.o \
./grapics/src/widgets/lv_checkbox.o \
./grapics/src/widgets/lv_dropdown.o \
./grapics/src/widgets/lv_img.o \
./grapics/src/widgets/lv_label.o \
./grapics/src/widgets/lv_line.o \
./grapics/src/widgets/lv_objx_templ.o \
./grapics/src/widgets/lv_roller.o \
./grapics/src/widgets/lv_slider.o \
./grapics/src/widgets/lv_switch.o \
./grapics/src/widgets/lv_table.o \
./grapics/src/widgets/lv_textarea.o 

C_DEPS += \
./grapics/src/widgets/lv_arc.d \
./grapics/src/widgets/lv_bar.d \
./grapics/src/widgets/lv_btn.d \
./grapics/src/widgets/lv_btnmatrix.d \
./grapics/src/widgets/lv_canvas.d \
./grapics/src/widgets/lv_checkbox.d \
./grapics/src/widgets/lv_dropdown.d \
./grapics/src/widgets/lv_img.d \
./grapics/src/widgets/lv_label.d \
./grapics/src/widgets/lv_line.d \
./grapics/src/widgets/lv_objx_templ.d \
./grapics/src/widgets/lv_roller.d \
./grapics/src/widgets/lv_slider.d \
./grapics/src/widgets/lv_switch.d \
./grapics/src/widgets/lv_table.d \
./grapics/src/widgets/lv_textarea.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/src/widgets/%.o grapics/src/widgets/%.su grapics/src/widgets/%.cyclo: ../grapics/src/widgets/%.c grapics/src/widgets/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-src-2f-widgets

clean-grapics-2f-src-2f-widgets:
	-$(RM) ./grapics/src/widgets/lv_arc.cyclo ./grapics/src/widgets/lv_arc.d ./grapics/src/widgets/lv_arc.o ./grapics/src/widgets/lv_arc.su ./grapics/src/widgets/lv_bar.cyclo ./grapics/src/widgets/lv_bar.d ./grapics/src/widgets/lv_bar.o ./grapics/src/widgets/lv_bar.su ./grapics/src/widgets/lv_btn.cyclo ./grapics/src/widgets/lv_btn.d ./grapics/src/widgets/lv_btn.o ./grapics/src/widgets/lv_btn.su ./grapics/src/widgets/lv_btnmatrix.cyclo ./grapics/src/widgets/lv_btnmatrix.d ./grapics/src/widgets/lv_btnmatrix.o ./grapics/src/widgets/lv_btnmatrix.su ./grapics/src/widgets/lv_canvas.cyclo ./grapics/src/widgets/lv_canvas.d ./grapics/src/widgets/lv_canvas.o ./grapics/src/widgets/lv_canvas.su ./grapics/src/widgets/lv_checkbox.cyclo ./grapics/src/widgets/lv_checkbox.d ./grapics/src/widgets/lv_checkbox.o ./grapics/src/widgets/lv_checkbox.su ./grapics/src/widgets/lv_dropdown.cyclo ./grapics/src/widgets/lv_dropdown.d ./grapics/src/widgets/lv_dropdown.o ./grapics/src/widgets/lv_dropdown.su ./grapics/src/widgets/lv_img.cyclo ./grapics/src/widgets/lv_img.d ./grapics/src/widgets/lv_img.o ./grapics/src/widgets/lv_img.su ./grapics/src/widgets/lv_label.cyclo ./grapics/src/widgets/lv_label.d ./grapics/src/widgets/lv_label.o ./grapics/src/widgets/lv_label.su ./grapics/src/widgets/lv_line.cyclo ./grapics/src/widgets/lv_line.d ./grapics/src/widgets/lv_line.o ./grapics/src/widgets/lv_line.su ./grapics/src/widgets/lv_objx_templ.cyclo ./grapics/src/widgets/lv_objx_templ.d ./grapics/src/widgets/lv_objx_templ.o ./grapics/src/widgets/lv_objx_templ.su ./grapics/src/widgets/lv_roller.cyclo ./grapics/src/widgets/lv_roller.d ./grapics/src/widgets/lv_roller.o ./grapics/src/widgets/lv_roller.su ./grapics/src/widgets/lv_slider.cyclo ./grapics/src/widgets/lv_slider.d ./grapics/src/widgets/lv_slider.o ./grapics/src/widgets/lv_slider.su ./grapics/src/widgets/lv_switch.cyclo ./grapics/src/widgets/lv_switch.d ./grapics/src/widgets/lv_switch.o ./grapics/src/widgets/lv_switch.su ./grapics/src/widgets/lv_table.cyclo ./grapics/src/widgets/lv_table.d ./grapics/src/widgets/lv_table.o ./grapics/src/widgets/lv_table.su ./grapics/src/widgets/lv_textarea.cyclo ./grapics/src/widgets/lv_textarea.d ./grapics/src/widgets/lv_textarea.o ./grapics/src/widgets/lv_textarea.su

.PHONY: clean-grapics-2f-src-2f-widgets

