################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/tests/src/test_cases/_test_template.c \
../grapics/tests/src/test_cases/test_arc.c \
../grapics/tests/src/test_cases/test_bar.c \
../grapics/tests/src/test_cases/test_checkbox.c \
../grapics/tests/src/test_cases/test_config.c \
../grapics/tests/src/test_cases/test_demo_stress.c \
../grapics/tests/src/test_cases/test_demo_widgets.c \
../grapics/tests/src/test_cases/test_dropdown.c \
../grapics/tests/src/test_cases/test_event.c \
../grapics/tests/src/test_cases/test_font_loader.c \
../grapics/tests/src/test_cases/test_fs.c \
../grapics/tests/src/test_cases/test_label.c \
../grapics/tests/src/test_cases/test_line.c \
../grapics/tests/src/test_cases/test_mem.c \
../grapics/tests/src/test_cases/test_obj_tree.c \
../grapics/tests/src/test_cases/test_roller.c \
../grapics/tests/src/test_cases/test_screen_load.c \
../grapics/tests/src/test_cases/test_slider.c \
../grapics/tests/src/test_cases/test_snapshot.c \
../grapics/tests/src/test_cases/test_span.c \
../grapics/tests/src/test_cases/test_style.c \
../grapics/tests/src/test_cases/test_switch.c \
../grapics/tests/src/test_cases/test_table.c \
../grapics/tests/src/test_cases/test_textarea.c \
../grapics/tests/src/test_cases/test_tiny_ttf.c \
../grapics/tests/src/test_cases/test_txt.c 

OBJS += \
./grapics/tests/src/test_cases/_test_template.o \
./grapics/tests/src/test_cases/test_arc.o \
./grapics/tests/src/test_cases/test_bar.o \
./grapics/tests/src/test_cases/test_checkbox.o \
./grapics/tests/src/test_cases/test_config.o \
./grapics/tests/src/test_cases/test_demo_stress.o \
./grapics/tests/src/test_cases/test_demo_widgets.o \
./grapics/tests/src/test_cases/test_dropdown.o \
./grapics/tests/src/test_cases/test_event.o \
./grapics/tests/src/test_cases/test_font_loader.o \
./grapics/tests/src/test_cases/test_fs.o \
./grapics/tests/src/test_cases/test_label.o \
./grapics/tests/src/test_cases/test_line.o \
./grapics/tests/src/test_cases/test_mem.o \
./grapics/tests/src/test_cases/test_obj_tree.o \
./grapics/tests/src/test_cases/test_roller.o \
./grapics/tests/src/test_cases/test_screen_load.o \
./grapics/tests/src/test_cases/test_slider.o \
./grapics/tests/src/test_cases/test_snapshot.o \
./grapics/tests/src/test_cases/test_span.o \
./grapics/tests/src/test_cases/test_style.o \
./grapics/tests/src/test_cases/test_switch.o \
./grapics/tests/src/test_cases/test_table.o \
./grapics/tests/src/test_cases/test_textarea.o \
./grapics/tests/src/test_cases/test_tiny_ttf.o \
./grapics/tests/src/test_cases/test_txt.o 

C_DEPS += \
./grapics/tests/src/test_cases/_test_template.d \
./grapics/tests/src/test_cases/test_arc.d \
./grapics/tests/src/test_cases/test_bar.d \
./grapics/tests/src/test_cases/test_checkbox.d \
./grapics/tests/src/test_cases/test_config.d \
./grapics/tests/src/test_cases/test_demo_stress.d \
./grapics/tests/src/test_cases/test_demo_widgets.d \
./grapics/tests/src/test_cases/test_dropdown.d \
./grapics/tests/src/test_cases/test_event.d \
./grapics/tests/src/test_cases/test_font_loader.d \
./grapics/tests/src/test_cases/test_fs.d \
./grapics/tests/src/test_cases/test_label.d \
./grapics/tests/src/test_cases/test_line.d \
./grapics/tests/src/test_cases/test_mem.d \
./grapics/tests/src/test_cases/test_obj_tree.d \
./grapics/tests/src/test_cases/test_roller.d \
./grapics/tests/src/test_cases/test_screen_load.d \
./grapics/tests/src/test_cases/test_slider.d \
./grapics/tests/src/test_cases/test_snapshot.d \
./grapics/tests/src/test_cases/test_span.d \
./grapics/tests/src/test_cases/test_style.d \
./grapics/tests/src/test_cases/test_switch.d \
./grapics/tests/src/test_cases/test_table.d \
./grapics/tests/src/test_cases/test_textarea.d \
./grapics/tests/src/test_cases/test_tiny_ttf.d \
./grapics/tests/src/test_cases/test_txt.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/tests/src/test_cases/%.o grapics/tests/src/test_cases/%.su grapics/tests/src/test_cases/%.cyclo: ../grapics/tests/src/test_cases/%.c grapics/tests/src/test_cases/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics/ui" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/grapics" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/Include" -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/GitHub/STM32_tof_camera/FreeRTOS/FreeRTOS-Kernel" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-tests-2f-src-2f-test_cases

clean-grapics-2f-tests-2f-src-2f-test_cases:
	-$(RM) ./grapics/tests/src/test_cases/_test_template.cyclo ./grapics/tests/src/test_cases/_test_template.d ./grapics/tests/src/test_cases/_test_template.o ./grapics/tests/src/test_cases/_test_template.su ./grapics/tests/src/test_cases/test_arc.cyclo ./grapics/tests/src/test_cases/test_arc.d ./grapics/tests/src/test_cases/test_arc.o ./grapics/tests/src/test_cases/test_arc.su ./grapics/tests/src/test_cases/test_bar.cyclo ./grapics/tests/src/test_cases/test_bar.d ./grapics/tests/src/test_cases/test_bar.o ./grapics/tests/src/test_cases/test_bar.su ./grapics/tests/src/test_cases/test_checkbox.cyclo ./grapics/tests/src/test_cases/test_checkbox.d ./grapics/tests/src/test_cases/test_checkbox.o ./grapics/tests/src/test_cases/test_checkbox.su ./grapics/tests/src/test_cases/test_config.cyclo ./grapics/tests/src/test_cases/test_config.d ./grapics/tests/src/test_cases/test_config.o ./grapics/tests/src/test_cases/test_config.su ./grapics/tests/src/test_cases/test_demo_stress.cyclo ./grapics/tests/src/test_cases/test_demo_stress.d ./grapics/tests/src/test_cases/test_demo_stress.o ./grapics/tests/src/test_cases/test_demo_stress.su ./grapics/tests/src/test_cases/test_demo_widgets.cyclo ./grapics/tests/src/test_cases/test_demo_widgets.d ./grapics/tests/src/test_cases/test_demo_widgets.o ./grapics/tests/src/test_cases/test_demo_widgets.su ./grapics/tests/src/test_cases/test_dropdown.cyclo ./grapics/tests/src/test_cases/test_dropdown.d ./grapics/tests/src/test_cases/test_dropdown.o ./grapics/tests/src/test_cases/test_dropdown.su ./grapics/tests/src/test_cases/test_event.cyclo ./grapics/tests/src/test_cases/test_event.d ./grapics/tests/src/test_cases/test_event.o ./grapics/tests/src/test_cases/test_event.su ./grapics/tests/src/test_cases/test_font_loader.cyclo ./grapics/tests/src/test_cases/test_font_loader.d ./grapics/tests/src/test_cases/test_font_loader.o ./grapics/tests/src/test_cases/test_font_loader.su ./grapics/tests/src/test_cases/test_fs.cyclo ./grapics/tests/src/test_cases/test_fs.d ./grapics/tests/src/test_cases/test_fs.o ./grapics/tests/src/test_cases/test_fs.su ./grapics/tests/src/test_cases/test_label.cyclo ./grapics/tests/src/test_cases/test_label.d ./grapics/tests/src/test_cases/test_label.o ./grapics/tests/src/test_cases/test_label.su ./grapics/tests/src/test_cases/test_line.cyclo ./grapics/tests/src/test_cases/test_line.d ./grapics/tests/src/test_cases/test_line.o ./grapics/tests/src/test_cases/test_line.su ./grapics/tests/src/test_cases/test_mem.cyclo ./grapics/tests/src/test_cases/test_mem.d ./grapics/tests/src/test_cases/test_mem.o ./grapics/tests/src/test_cases/test_mem.su ./grapics/tests/src/test_cases/test_obj_tree.cyclo ./grapics/tests/src/test_cases/test_obj_tree.d ./grapics/tests/src/test_cases/test_obj_tree.o ./grapics/tests/src/test_cases/test_obj_tree.su ./grapics/tests/src/test_cases/test_roller.cyclo ./grapics/tests/src/test_cases/test_roller.d ./grapics/tests/src/test_cases/test_roller.o ./grapics/tests/src/test_cases/test_roller.su ./grapics/tests/src/test_cases/test_screen_load.cyclo ./grapics/tests/src/test_cases/test_screen_load.d ./grapics/tests/src/test_cases/test_screen_load.o ./grapics/tests/src/test_cases/test_screen_load.su ./grapics/tests/src/test_cases/test_slider.cyclo ./grapics/tests/src/test_cases/test_slider.d ./grapics/tests/src/test_cases/test_slider.o ./grapics/tests/src/test_cases/test_slider.su ./grapics/tests/src/test_cases/test_snapshot.cyclo ./grapics/tests/src/test_cases/test_snapshot.d ./grapics/tests/src/test_cases/test_snapshot.o ./grapics/tests/src/test_cases/test_snapshot.su ./grapics/tests/src/test_cases/test_span.cyclo ./grapics/tests/src/test_cases/test_span.d ./grapics/tests/src/test_cases/test_span.o ./grapics/tests/src/test_cases/test_span.su ./grapics/tests/src/test_cases/test_style.cyclo ./grapics/tests/src/test_cases/test_style.d ./grapics/tests/src/test_cases/test_style.o ./grapics/tests/src/test_cases/test_style.su ./grapics/tests/src/test_cases/test_switch.cyclo ./grapics/tests/src/test_cases/test_switch.d ./grapics/tests/src/test_cases/test_switch.o ./grapics/tests/src/test_cases/test_switch.su ./grapics/tests/src/test_cases/test_table.cyclo ./grapics/tests/src/test_cases/test_table.d ./grapics/tests/src/test_cases/test_table.o ./grapics/tests/src/test_cases/test_table.su ./grapics/tests/src/test_cases/test_textarea.cyclo ./grapics/tests/src/test_cases/test_textarea.d ./grapics/tests/src/test_cases/test_textarea.o ./grapics/tests/src/test_cases/test_textarea.su ./grapics/tests/src/test_cases/test_tiny_ttf.cyclo ./grapics/tests/src/test_cases/test_tiny_ttf.d ./grapics/tests/src/test_cases/test_tiny_ttf.o ./grapics/tests/src/test_cases/test_tiny_ttf.su ./grapics/tests/src/test_cases/test_txt.cyclo ./grapics/tests/src/test_cases/test_txt.d ./grapics/tests/src/test_cases/test_txt.o ./grapics/tests/src/test_cases/test_txt.su

.PHONY: clean-grapics-2f-tests-2f-src-2f-test_cases

