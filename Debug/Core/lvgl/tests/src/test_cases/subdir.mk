################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lvgl/tests/src/test_cases/_test_template.c \
../Core/lvgl/tests/src/test_cases/test_align_flex.c \
../Core/lvgl/tests/src/test_cases/test_anim.c \
../Core/lvgl/tests/src/test_cases/test_anim_timeline.c \
../Core/lvgl/tests/src/test_cases/test_area.c \
../Core/lvgl/tests/src/test_cases/test_array.c \
../Core/lvgl/tests/src/test_cases/test_async.c \
../Core/lvgl/tests/src/test_cases/test_bindings.c \
../Core/lvgl/tests/src/test_cases/test_circle_buf.c \
../Core/lvgl/tests/src/test_cases/test_click.c \
../Core/lvgl/tests/src/test_cases/test_color.c \
../Core/lvgl/tests/src/test_cases/test_config.c \
../Core/lvgl/tests/src/test_cases/test_demo_stress.c \
../Core/lvgl/tests/src/test_cases/test_demo_vector_graphic.c \
../Core/lvgl/tests/src/test_cases/test_demo_widgets.c \
../Core/lvgl/tests/src/test_cases/test_display.c \
../Core/lvgl/tests/src/test_cases/test_draw_buf.c \
../Core/lvgl/tests/src/test_cases/test_event.c \
../Core/lvgl/tests/src/test_cases/test_event_trickle.c \
../Core/lvgl/tests/src/test_cases/test_file_explorer.c \
../Core/lvgl/tests/src/test_cases/test_flex_grow.c \
../Core/lvgl/tests/src/test_cases/test_font_loader.c \
../Core/lvgl/tests/src/test_cases/test_font_manager.c \
../Core/lvgl/tests/src/test_cases/test_fs.c \
../Core/lvgl/tests/src/test_cases/test_gesture_pinch.c \
../Core/lvgl/tests/src/test_cases/test_grid.c \
../Core/lvgl/tests/src/test_cases/test_grid_fr.c \
../Core/lvgl/tests/src/test_cases/test_gridnav.c \
../Core/lvgl/tests/src/test_cases/test_group.c \
../Core/lvgl/tests/src/test_cases/test_hover.c \
../Core/lvgl/tests/src/test_cases/test_indev.c \
../Core/lvgl/tests/src/test_cases/test_indev_event.c \
../Core/lvgl/tests/src/test_cases/test_indev_key_remap.c \
../Core/lvgl/tests/src/test_cases/test_lcd.c \
../Core/lvgl/tests/src/test_cases/test_ll.c \
../Core/lvgl/tests/src/test_cases/test_margin_align.c \
../Core/lvgl/tests/src/test_cases/test_margin_flex.c \
../Core/lvgl/tests/src/test_cases/test_margin_grid.c \
../Core/lvgl/tests/src/test_cases/test_math.c \
../Core/lvgl/tests/src/test_cases/test_mem.c \
../Core/lvgl/tests/src/test_cases/test_observer.c \
../Core/lvgl/tests/src/test_cases/test_palette.c \
../Core/lvgl/tests/src/test_cases/test_profiler.c \
../Core/lvgl/tests/src/test_cases/test_recolor.c \
../Core/lvgl/tests/src/test_cases/test_screen_load.c \
../Core/lvgl/tests/src/test_cases/test_snapshot.c \
../Core/lvgl/tests/src/test_cases/test_style.c \
../Core/lvgl/tests/src/test_cases/test_svg.c \
../Core/lvgl/tests/src/test_cases/test_svg_anim.c \
../Core/lvgl/tests/src/test_cases/test_svg_file.c \
../Core/lvgl/tests/src/test_cases/test_theme.c \
../Core/lvgl/tests/src/test_cases/test_tick.c \
../Core/lvgl/tests/src/test_cases/test_translation.c \
../Core/lvgl/tests/src/test_cases/test_tree.c \
../Core/lvgl/tests/src/test_cases/test_txt.c \
../Core/lvgl/tests/src/test_cases/test_utils.c 

OBJS += \
./Core/lvgl/tests/src/test_cases/_test_template.o \
./Core/lvgl/tests/src/test_cases/test_align_flex.o \
./Core/lvgl/tests/src/test_cases/test_anim.o \
./Core/lvgl/tests/src/test_cases/test_anim_timeline.o \
./Core/lvgl/tests/src/test_cases/test_area.o \
./Core/lvgl/tests/src/test_cases/test_array.o \
./Core/lvgl/tests/src/test_cases/test_async.o \
./Core/lvgl/tests/src/test_cases/test_bindings.o \
./Core/lvgl/tests/src/test_cases/test_circle_buf.o \
./Core/lvgl/tests/src/test_cases/test_click.o \
./Core/lvgl/tests/src/test_cases/test_color.o \
./Core/lvgl/tests/src/test_cases/test_config.o \
./Core/lvgl/tests/src/test_cases/test_demo_stress.o \
./Core/lvgl/tests/src/test_cases/test_demo_vector_graphic.o \
./Core/lvgl/tests/src/test_cases/test_demo_widgets.o \
./Core/lvgl/tests/src/test_cases/test_display.o \
./Core/lvgl/tests/src/test_cases/test_draw_buf.o \
./Core/lvgl/tests/src/test_cases/test_event.o \
./Core/lvgl/tests/src/test_cases/test_event_trickle.o \
./Core/lvgl/tests/src/test_cases/test_file_explorer.o \
./Core/lvgl/tests/src/test_cases/test_flex_grow.o \
./Core/lvgl/tests/src/test_cases/test_font_loader.o \
./Core/lvgl/tests/src/test_cases/test_font_manager.o \
./Core/lvgl/tests/src/test_cases/test_fs.o \
./Core/lvgl/tests/src/test_cases/test_gesture_pinch.o \
./Core/lvgl/tests/src/test_cases/test_grid.o \
./Core/lvgl/tests/src/test_cases/test_grid_fr.o \
./Core/lvgl/tests/src/test_cases/test_gridnav.o \
./Core/lvgl/tests/src/test_cases/test_group.o \
./Core/lvgl/tests/src/test_cases/test_hover.o \
./Core/lvgl/tests/src/test_cases/test_indev.o \
./Core/lvgl/tests/src/test_cases/test_indev_event.o \
./Core/lvgl/tests/src/test_cases/test_indev_key_remap.o \
./Core/lvgl/tests/src/test_cases/test_lcd.o \
./Core/lvgl/tests/src/test_cases/test_ll.o \
./Core/lvgl/tests/src/test_cases/test_margin_align.o \
./Core/lvgl/tests/src/test_cases/test_margin_flex.o \
./Core/lvgl/tests/src/test_cases/test_margin_grid.o \
./Core/lvgl/tests/src/test_cases/test_math.o \
./Core/lvgl/tests/src/test_cases/test_mem.o \
./Core/lvgl/tests/src/test_cases/test_observer.o \
./Core/lvgl/tests/src/test_cases/test_palette.o \
./Core/lvgl/tests/src/test_cases/test_profiler.o \
./Core/lvgl/tests/src/test_cases/test_recolor.o \
./Core/lvgl/tests/src/test_cases/test_screen_load.o \
./Core/lvgl/tests/src/test_cases/test_snapshot.o \
./Core/lvgl/tests/src/test_cases/test_style.o \
./Core/lvgl/tests/src/test_cases/test_svg.o \
./Core/lvgl/tests/src/test_cases/test_svg_anim.o \
./Core/lvgl/tests/src/test_cases/test_svg_file.o \
./Core/lvgl/tests/src/test_cases/test_theme.o \
./Core/lvgl/tests/src/test_cases/test_tick.o \
./Core/lvgl/tests/src/test_cases/test_translation.o \
./Core/lvgl/tests/src/test_cases/test_tree.o \
./Core/lvgl/tests/src/test_cases/test_txt.o \
./Core/lvgl/tests/src/test_cases/test_utils.o 

C_DEPS += \
./Core/lvgl/tests/src/test_cases/_test_template.d \
./Core/lvgl/tests/src/test_cases/test_align_flex.d \
./Core/lvgl/tests/src/test_cases/test_anim.d \
./Core/lvgl/tests/src/test_cases/test_anim_timeline.d \
./Core/lvgl/tests/src/test_cases/test_area.d \
./Core/lvgl/tests/src/test_cases/test_array.d \
./Core/lvgl/tests/src/test_cases/test_async.d \
./Core/lvgl/tests/src/test_cases/test_bindings.d \
./Core/lvgl/tests/src/test_cases/test_circle_buf.d \
./Core/lvgl/tests/src/test_cases/test_click.d \
./Core/lvgl/tests/src/test_cases/test_color.d \
./Core/lvgl/tests/src/test_cases/test_config.d \
./Core/lvgl/tests/src/test_cases/test_demo_stress.d \
./Core/lvgl/tests/src/test_cases/test_demo_vector_graphic.d \
./Core/lvgl/tests/src/test_cases/test_demo_widgets.d \
./Core/lvgl/tests/src/test_cases/test_display.d \
./Core/lvgl/tests/src/test_cases/test_draw_buf.d \
./Core/lvgl/tests/src/test_cases/test_event.d \
./Core/lvgl/tests/src/test_cases/test_event_trickle.d \
./Core/lvgl/tests/src/test_cases/test_file_explorer.d \
./Core/lvgl/tests/src/test_cases/test_flex_grow.d \
./Core/lvgl/tests/src/test_cases/test_font_loader.d \
./Core/lvgl/tests/src/test_cases/test_font_manager.d \
./Core/lvgl/tests/src/test_cases/test_fs.d \
./Core/lvgl/tests/src/test_cases/test_gesture_pinch.d \
./Core/lvgl/tests/src/test_cases/test_grid.d \
./Core/lvgl/tests/src/test_cases/test_grid_fr.d \
./Core/lvgl/tests/src/test_cases/test_gridnav.d \
./Core/lvgl/tests/src/test_cases/test_group.d \
./Core/lvgl/tests/src/test_cases/test_hover.d \
./Core/lvgl/tests/src/test_cases/test_indev.d \
./Core/lvgl/tests/src/test_cases/test_indev_event.d \
./Core/lvgl/tests/src/test_cases/test_indev_key_remap.d \
./Core/lvgl/tests/src/test_cases/test_lcd.d \
./Core/lvgl/tests/src/test_cases/test_ll.d \
./Core/lvgl/tests/src/test_cases/test_margin_align.d \
./Core/lvgl/tests/src/test_cases/test_margin_flex.d \
./Core/lvgl/tests/src/test_cases/test_margin_grid.d \
./Core/lvgl/tests/src/test_cases/test_math.d \
./Core/lvgl/tests/src/test_cases/test_mem.d \
./Core/lvgl/tests/src/test_cases/test_observer.d \
./Core/lvgl/tests/src/test_cases/test_palette.d \
./Core/lvgl/tests/src/test_cases/test_profiler.d \
./Core/lvgl/tests/src/test_cases/test_recolor.d \
./Core/lvgl/tests/src/test_cases/test_screen_load.d \
./Core/lvgl/tests/src/test_cases/test_snapshot.d \
./Core/lvgl/tests/src/test_cases/test_style.d \
./Core/lvgl/tests/src/test_cases/test_svg.d \
./Core/lvgl/tests/src/test_cases/test_svg_anim.d \
./Core/lvgl/tests/src/test_cases/test_svg_file.d \
./Core/lvgl/tests/src/test_cases/test_theme.d \
./Core/lvgl/tests/src/test_cases/test_tick.d \
./Core/lvgl/tests/src/test_cases/test_translation.d \
./Core/lvgl/tests/src/test_cases/test_tree.d \
./Core/lvgl/tests/src/test_cases/test_txt.d \
./Core/lvgl/tests/src/test_cases/test_utils.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lvgl/tests/src/test_cases/%.o Core/lvgl/tests/src/test_cases/%.su Core/lvgl/tests/src/test_cases/%.cyclo: ../Core/lvgl/tests/src/test_cases/%.c Core/lvgl/tests/src/test_cases/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/lvgl" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lvgl-2f-tests-2f-src-2f-test_cases

clean-Core-2f-lvgl-2f-tests-2f-src-2f-test_cases:
	-$(RM) ./Core/lvgl/tests/src/test_cases/_test_template.cyclo ./Core/lvgl/tests/src/test_cases/_test_template.d ./Core/lvgl/tests/src/test_cases/_test_template.o ./Core/lvgl/tests/src/test_cases/_test_template.su ./Core/lvgl/tests/src/test_cases/test_align_flex.cyclo ./Core/lvgl/tests/src/test_cases/test_align_flex.d ./Core/lvgl/tests/src/test_cases/test_align_flex.o ./Core/lvgl/tests/src/test_cases/test_align_flex.su ./Core/lvgl/tests/src/test_cases/test_anim.cyclo ./Core/lvgl/tests/src/test_cases/test_anim.d ./Core/lvgl/tests/src/test_cases/test_anim.o ./Core/lvgl/tests/src/test_cases/test_anim.su ./Core/lvgl/tests/src/test_cases/test_anim_timeline.cyclo ./Core/lvgl/tests/src/test_cases/test_anim_timeline.d ./Core/lvgl/tests/src/test_cases/test_anim_timeline.o ./Core/lvgl/tests/src/test_cases/test_anim_timeline.su ./Core/lvgl/tests/src/test_cases/test_area.cyclo ./Core/lvgl/tests/src/test_cases/test_area.d ./Core/lvgl/tests/src/test_cases/test_area.o ./Core/lvgl/tests/src/test_cases/test_area.su ./Core/lvgl/tests/src/test_cases/test_array.cyclo ./Core/lvgl/tests/src/test_cases/test_array.d ./Core/lvgl/tests/src/test_cases/test_array.o ./Core/lvgl/tests/src/test_cases/test_array.su ./Core/lvgl/tests/src/test_cases/test_async.cyclo ./Core/lvgl/tests/src/test_cases/test_async.d ./Core/lvgl/tests/src/test_cases/test_async.o ./Core/lvgl/tests/src/test_cases/test_async.su ./Core/lvgl/tests/src/test_cases/test_bindings.cyclo ./Core/lvgl/tests/src/test_cases/test_bindings.d ./Core/lvgl/tests/src/test_cases/test_bindings.o ./Core/lvgl/tests/src/test_cases/test_bindings.su ./Core/lvgl/tests/src/test_cases/test_circle_buf.cyclo ./Core/lvgl/tests/src/test_cases/test_circle_buf.d ./Core/lvgl/tests/src/test_cases/test_circle_buf.o ./Core/lvgl/tests/src/test_cases/test_circle_buf.su ./Core/lvgl/tests/src/test_cases/test_click.cyclo ./Core/lvgl/tests/src/test_cases/test_click.d ./Core/lvgl/tests/src/test_cases/test_click.o ./Core/lvgl/tests/src/test_cases/test_click.su ./Core/lvgl/tests/src/test_cases/test_color.cyclo ./Core/lvgl/tests/src/test_cases/test_color.d ./Core/lvgl/tests/src/test_cases/test_color.o ./Core/lvgl/tests/src/test_cases/test_color.su ./Core/lvgl/tests/src/test_cases/test_config.cyclo ./Core/lvgl/tests/src/test_cases/test_config.d ./Core/lvgl/tests/src/test_cases/test_config.o ./Core/lvgl/tests/src/test_cases/test_config.su ./Core/lvgl/tests/src/test_cases/test_demo_stress.cyclo ./Core/lvgl/tests/src/test_cases/test_demo_stress.d ./Core/lvgl/tests/src/test_cases/test_demo_stress.o ./Core/lvgl/tests/src/test_cases/test_demo_stress.su ./Core/lvgl/tests/src/test_cases/test_demo_vector_graphic.cyclo ./Core/lvgl/tests/src/test_cases/test_demo_vector_graphic.d ./Core/lvgl/tests/src/test_cases/test_demo_vector_graphic.o ./Core/lvgl/tests/src/test_cases/test_demo_vector_graphic.su ./Core/lvgl/tests/src/test_cases/test_demo_widgets.cyclo ./Core/lvgl/tests/src/test_cases/test_demo_widgets.d ./Core/lvgl/tests/src/test_cases/test_demo_widgets.o ./Core/lvgl/tests/src/test_cases/test_demo_widgets.su ./Core/lvgl/tests/src/test_cases/test_display.cyclo ./Core/lvgl/tests/src/test_cases/test_display.d ./Core/lvgl/tests/src/test_cases/test_display.o ./Core/lvgl/tests/src/test_cases/test_display.su ./Core/lvgl/tests/src/test_cases/test_draw_buf.cyclo ./Core/lvgl/tests/src/test_cases/test_draw_buf.d ./Core/lvgl/tests/src/test_cases/test_draw_buf.o ./Core/lvgl/tests/src/test_cases/test_draw_buf.su ./Core/lvgl/tests/src/test_cases/test_event.cyclo ./Core/lvgl/tests/src/test_cases/test_event.d ./Core/lvgl/tests/src/test_cases/test_event.o ./Core/lvgl/tests/src/test_cases/test_event.su ./Core/lvgl/tests/src/test_cases/test_event_trickle.cyclo ./Core/lvgl/tests/src/test_cases/test_event_trickle.d ./Core/lvgl/tests/src/test_cases/test_event_trickle.o ./Core/lvgl/tests/src/test_cases/test_event_trickle.su ./Core/lvgl/tests/src/test_cases/test_file_explorer.cyclo ./Core/lvgl/tests/src/test_cases/test_file_explorer.d ./Core/lvgl/tests/src/test_cases/test_file_explorer.o ./Core/lvgl/tests/src/test_cases/test_file_explorer.su ./Core/lvgl/tests/src/test_cases/test_flex_grow.cyclo ./Core/lvgl/tests/src/test_cases/test_flex_grow.d ./Core/lvgl/tests/src/test_cases/test_flex_grow.o ./Core/lvgl/tests/src/test_cases/test_flex_grow.su ./Core/lvgl/tests/src/test_cases/test_font_loader.cyclo ./Core/lvgl/tests/src/test_cases/test_font_loader.d ./Core/lvgl/tests/src/test_cases/test_font_loader.o ./Core/lvgl/tests/src/test_cases/test_font_loader.su ./Core/lvgl/tests/src/test_cases/test_font_manager.cyclo ./Core/lvgl/tests/src/test_cases/test_font_manager.d ./Core/lvgl/tests/src/test_cases/test_font_manager.o ./Core/lvgl/tests/src/test_cases/test_font_manager.su ./Core/lvgl/tests/src/test_cases/test_fs.cyclo ./Core/lvgl/tests/src/test_cases/test_fs.d ./Core/lvgl/tests/src/test_cases/test_fs.o ./Core/lvgl/tests/src/test_cases/test_fs.su ./Core/lvgl/tests/src/test_cases/test_gesture_pinch.cyclo ./Core/lvgl/tests/src/test_cases/test_gesture_pinch.d ./Core/lvgl/tests/src/test_cases/test_gesture_pinch.o ./Core/lvgl/tests/src/test_cases/test_gesture_pinch.su ./Core/lvgl/tests/src/test_cases/test_grid.cyclo ./Core/lvgl/tests/src/test_cases/test_grid.d ./Core/lvgl/tests/src/test_cases/test_grid.o ./Core/lvgl/tests/src/test_cases/test_grid.su ./Core/lvgl/tests/src/test_cases/test_grid_fr.cyclo ./Core/lvgl/tests/src/test_cases/test_grid_fr.d ./Core/lvgl/tests/src/test_cases/test_grid_fr.o ./Core/lvgl/tests/src/test_cases/test_grid_fr.su ./Core/lvgl/tests/src/test_cases/test_gridnav.cyclo ./Core/lvgl/tests/src/test_cases/test_gridnav.d ./Core/lvgl/tests/src/test_cases/test_gridnav.o ./Core/lvgl/tests/src/test_cases/test_gridnav.su ./Core/lvgl/tests/src/test_cases/test_group.cyclo ./Core/lvgl/tests/src/test_cases/test_group.d ./Core/lvgl/tests/src/test_cases/test_group.o ./Core/lvgl/tests/src/test_cases/test_group.su ./Core/lvgl/tests/src/test_cases/test_hover.cyclo ./Core/lvgl/tests/src/test_cases/test_hover.d
	-$(RM) ./Core/lvgl/tests/src/test_cases/test_hover.o ./Core/lvgl/tests/src/test_cases/test_hover.su ./Core/lvgl/tests/src/test_cases/test_indev.cyclo ./Core/lvgl/tests/src/test_cases/test_indev.d ./Core/lvgl/tests/src/test_cases/test_indev.o ./Core/lvgl/tests/src/test_cases/test_indev.su ./Core/lvgl/tests/src/test_cases/test_indev_event.cyclo ./Core/lvgl/tests/src/test_cases/test_indev_event.d ./Core/lvgl/tests/src/test_cases/test_indev_event.o ./Core/lvgl/tests/src/test_cases/test_indev_event.su ./Core/lvgl/tests/src/test_cases/test_indev_key_remap.cyclo ./Core/lvgl/tests/src/test_cases/test_indev_key_remap.d ./Core/lvgl/tests/src/test_cases/test_indev_key_remap.o ./Core/lvgl/tests/src/test_cases/test_indev_key_remap.su ./Core/lvgl/tests/src/test_cases/test_lcd.cyclo ./Core/lvgl/tests/src/test_cases/test_lcd.d ./Core/lvgl/tests/src/test_cases/test_lcd.o ./Core/lvgl/tests/src/test_cases/test_lcd.su ./Core/lvgl/tests/src/test_cases/test_ll.cyclo ./Core/lvgl/tests/src/test_cases/test_ll.d ./Core/lvgl/tests/src/test_cases/test_ll.o ./Core/lvgl/tests/src/test_cases/test_ll.su ./Core/lvgl/tests/src/test_cases/test_margin_align.cyclo ./Core/lvgl/tests/src/test_cases/test_margin_align.d ./Core/lvgl/tests/src/test_cases/test_margin_align.o ./Core/lvgl/tests/src/test_cases/test_margin_align.su ./Core/lvgl/tests/src/test_cases/test_margin_flex.cyclo ./Core/lvgl/tests/src/test_cases/test_margin_flex.d ./Core/lvgl/tests/src/test_cases/test_margin_flex.o ./Core/lvgl/tests/src/test_cases/test_margin_flex.su ./Core/lvgl/tests/src/test_cases/test_margin_grid.cyclo ./Core/lvgl/tests/src/test_cases/test_margin_grid.d ./Core/lvgl/tests/src/test_cases/test_margin_grid.o ./Core/lvgl/tests/src/test_cases/test_margin_grid.su ./Core/lvgl/tests/src/test_cases/test_math.cyclo ./Core/lvgl/tests/src/test_cases/test_math.d ./Core/lvgl/tests/src/test_cases/test_math.o ./Core/lvgl/tests/src/test_cases/test_math.su ./Core/lvgl/tests/src/test_cases/test_mem.cyclo ./Core/lvgl/tests/src/test_cases/test_mem.d ./Core/lvgl/tests/src/test_cases/test_mem.o ./Core/lvgl/tests/src/test_cases/test_mem.su ./Core/lvgl/tests/src/test_cases/test_observer.cyclo ./Core/lvgl/tests/src/test_cases/test_observer.d ./Core/lvgl/tests/src/test_cases/test_observer.o ./Core/lvgl/tests/src/test_cases/test_observer.su ./Core/lvgl/tests/src/test_cases/test_palette.cyclo ./Core/lvgl/tests/src/test_cases/test_palette.d ./Core/lvgl/tests/src/test_cases/test_palette.o ./Core/lvgl/tests/src/test_cases/test_palette.su ./Core/lvgl/tests/src/test_cases/test_profiler.cyclo ./Core/lvgl/tests/src/test_cases/test_profiler.d ./Core/lvgl/tests/src/test_cases/test_profiler.o ./Core/lvgl/tests/src/test_cases/test_profiler.su ./Core/lvgl/tests/src/test_cases/test_recolor.cyclo ./Core/lvgl/tests/src/test_cases/test_recolor.d ./Core/lvgl/tests/src/test_cases/test_recolor.o ./Core/lvgl/tests/src/test_cases/test_recolor.su ./Core/lvgl/tests/src/test_cases/test_screen_load.cyclo ./Core/lvgl/tests/src/test_cases/test_screen_load.d ./Core/lvgl/tests/src/test_cases/test_screen_load.o ./Core/lvgl/tests/src/test_cases/test_screen_load.su ./Core/lvgl/tests/src/test_cases/test_snapshot.cyclo ./Core/lvgl/tests/src/test_cases/test_snapshot.d ./Core/lvgl/tests/src/test_cases/test_snapshot.o ./Core/lvgl/tests/src/test_cases/test_snapshot.su ./Core/lvgl/tests/src/test_cases/test_style.cyclo ./Core/lvgl/tests/src/test_cases/test_style.d ./Core/lvgl/tests/src/test_cases/test_style.o ./Core/lvgl/tests/src/test_cases/test_style.su ./Core/lvgl/tests/src/test_cases/test_svg.cyclo ./Core/lvgl/tests/src/test_cases/test_svg.d ./Core/lvgl/tests/src/test_cases/test_svg.o ./Core/lvgl/tests/src/test_cases/test_svg.su ./Core/lvgl/tests/src/test_cases/test_svg_anim.cyclo ./Core/lvgl/tests/src/test_cases/test_svg_anim.d ./Core/lvgl/tests/src/test_cases/test_svg_anim.o ./Core/lvgl/tests/src/test_cases/test_svg_anim.su ./Core/lvgl/tests/src/test_cases/test_svg_file.cyclo ./Core/lvgl/tests/src/test_cases/test_svg_file.d ./Core/lvgl/tests/src/test_cases/test_svg_file.o ./Core/lvgl/tests/src/test_cases/test_svg_file.su ./Core/lvgl/tests/src/test_cases/test_theme.cyclo ./Core/lvgl/tests/src/test_cases/test_theme.d ./Core/lvgl/tests/src/test_cases/test_theme.o ./Core/lvgl/tests/src/test_cases/test_theme.su ./Core/lvgl/tests/src/test_cases/test_tick.cyclo ./Core/lvgl/tests/src/test_cases/test_tick.d ./Core/lvgl/tests/src/test_cases/test_tick.o ./Core/lvgl/tests/src/test_cases/test_tick.su ./Core/lvgl/tests/src/test_cases/test_translation.cyclo ./Core/lvgl/tests/src/test_cases/test_translation.d ./Core/lvgl/tests/src/test_cases/test_translation.o ./Core/lvgl/tests/src/test_cases/test_translation.su ./Core/lvgl/tests/src/test_cases/test_tree.cyclo ./Core/lvgl/tests/src/test_cases/test_tree.d ./Core/lvgl/tests/src/test_cases/test_tree.o ./Core/lvgl/tests/src/test_cases/test_tree.su ./Core/lvgl/tests/src/test_cases/test_txt.cyclo ./Core/lvgl/tests/src/test_cases/test_txt.d ./Core/lvgl/tests/src/test_cases/test_txt.o ./Core/lvgl/tests/src/test_cases/test_txt.su ./Core/lvgl/tests/src/test_cases/test_utils.cyclo ./Core/lvgl/tests/src/test_cases/test_utils.d ./Core/lvgl/tests/src/test_cases/test_utils.o ./Core/lvgl/tests/src/test_cases/test_utils.su

.PHONY: clean-Core-2f-lvgl-2f-tests-2f-src-2f-test_cases

