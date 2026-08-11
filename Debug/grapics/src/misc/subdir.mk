################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/src/misc/lv_anim.c \
../grapics/src/misc/lv_anim_timeline.c \
../grapics/src/misc/lv_area.c \
../grapics/src/misc/lv_async.c \
../grapics/src/misc/lv_bidi.c \
../grapics/src/misc/lv_color.c \
../grapics/src/misc/lv_fs.c \
../grapics/src/misc/lv_gc.c \
../grapics/src/misc/lv_ll.c \
../grapics/src/misc/lv_log.c \
../grapics/src/misc/lv_lru.c \
../grapics/src/misc/lv_math.c \
../grapics/src/misc/lv_mem.c \
../grapics/src/misc/lv_printf.c \
../grapics/src/misc/lv_style.c \
../grapics/src/misc/lv_style_gen.c \
../grapics/src/misc/lv_templ.c \
../grapics/src/misc/lv_timer.c \
../grapics/src/misc/lv_tlsf.c \
../grapics/src/misc/lv_txt.c \
../grapics/src/misc/lv_txt_ap.c \
../grapics/src/misc/lv_utils.c 

OBJS += \
./grapics/src/misc/lv_anim.o \
./grapics/src/misc/lv_anim_timeline.o \
./grapics/src/misc/lv_area.o \
./grapics/src/misc/lv_async.o \
./grapics/src/misc/lv_bidi.o \
./grapics/src/misc/lv_color.o \
./grapics/src/misc/lv_fs.o \
./grapics/src/misc/lv_gc.o \
./grapics/src/misc/lv_ll.o \
./grapics/src/misc/lv_log.o \
./grapics/src/misc/lv_lru.o \
./grapics/src/misc/lv_math.o \
./grapics/src/misc/lv_mem.o \
./grapics/src/misc/lv_printf.o \
./grapics/src/misc/lv_style.o \
./grapics/src/misc/lv_style_gen.o \
./grapics/src/misc/lv_templ.o \
./grapics/src/misc/lv_timer.o \
./grapics/src/misc/lv_tlsf.o \
./grapics/src/misc/lv_txt.o \
./grapics/src/misc/lv_txt_ap.o \
./grapics/src/misc/lv_utils.o 

C_DEPS += \
./grapics/src/misc/lv_anim.d \
./grapics/src/misc/lv_anim_timeline.d \
./grapics/src/misc/lv_area.d \
./grapics/src/misc/lv_async.d \
./grapics/src/misc/lv_bidi.d \
./grapics/src/misc/lv_color.d \
./grapics/src/misc/lv_fs.d \
./grapics/src/misc/lv_gc.d \
./grapics/src/misc/lv_ll.d \
./grapics/src/misc/lv_log.d \
./grapics/src/misc/lv_lru.d \
./grapics/src/misc/lv_math.d \
./grapics/src/misc/lv_mem.d \
./grapics/src/misc/lv_printf.d \
./grapics/src/misc/lv_style.d \
./grapics/src/misc/lv_style_gen.d \
./grapics/src/misc/lv_templ.d \
./grapics/src/misc/lv_timer.d \
./grapics/src/misc/lv_tlsf.d \
./grapics/src/misc/lv_txt.d \
./grapics/src/misc/lv_txt_ap.d \
./grapics/src/misc/lv_utils.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/src/misc/%.o grapics/src/misc/%.su grapics/src/misc/%.cyclo: ../grapics/src/misc/%.c grapics/src/misc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-src-2f-misc

clean-grapics-2f-src-2f-misc:
	-$(RM) ./grapics/src/misc/lv_anim.cyclo ./grapics/src/misc/lv_anim.d ./grapics/src/misc/lv_anim.o ./grapics/src/misc/lv_anim.su ./grapics/src/misc/lv_anim_timeline.cyclo ./grapics/src/misc/lv_anim_timeline.d ./grapics/src/misc/lv_anim_timeline.o ./grapics/src/misc/lv_anim_timeline.su ./grapics/src/misc/lv_area.cyclo ./grapics/src/misc/lv_area.d ./grapics/src/misc/lv_area.o ./grapics/src/misc/lv_area.su ./grapics/src/misc/lv_async.cyclo ./grapics/src/misc/lv_async.d ./grapics/src/misc/lv_async.o ./grapics/src/misc/lv_async.su ./grapics/src/misc/lv_bidi.cyclo ./grapics/src/misc/lv_bidi.d ./grapics/src/misc/lv_bidi.o ./grapics/src/misc/lv_bidi.su ./grapics/src/misc/lv_color.cyclo ./grapics/src/misc/lv_color.d ./grapics/src/misc/lv_color.o ./grapics/src/misc/lv_color.su ./grapics/src/misc/lv_fs.cyclo ./grapics/src/misc/lv_fs.d ./grapics/src/misc/lv_fs.o ./grapics/src/misc/lv_fs.su ./grapics/src/misc/lv_gc.cyclo ./grapics/src/misc/lv_gc.d ./grapics/src/misc/lv_gc.o ./grapics/src/misc/lv_gc.su ./grapics/src/misc/lv_ll.cyclo ./grapics/src/misc/lv_ll.d ./grapics/src/misc/lv_ll.o ./grapics/src/misc/lv_ll.su ./grapics/src/misc/lv_log.cyclo ./grapics/src/misc/lv_log.d ./grapics/src/misc/lv_log.o ./grapics/src/misc/lv_log.su ./grapics/src/misc/lv_lru.cyclo ./grapics/src/misc/lv_lru.d ./grapics/src/misc/lv_lru.o ./grapics/src/misc/lv_lru.su ./grapics/src/misc/lv_math.cyclo ./grapics/src/misc/lv_math.d ./grapics/src/misc/lv_math.o ./grapics/src/misc/lv_math.su ./grapics/src/misc/lv_mem.cyclo ./grapics/src/misc/lv_mem.d ./grapics/src/misc/lv_mem.o ./grapics/src/misc/lv_mem.su ./grapics/src/misc/lv_printf.cyclo ./grapics/src/misc/lv_printf.d ./grapics/src/misc/lv_printf.o ./grapics/src/misc/lv_printf.su ./grapics/src/misc/lv_style.cyclo ./grapics/src/misc/lv_style.d ./grapics/src/misc/lv_style.o ./grapics/src/misc/lv_style.su ./grapics/src/misc/lv_style_gen.cyclo ./grapics/src/misc/lv_style_gen.d ./grapics/src/misc/lv_style_gen.o ./grapics/src/misc/lv_style_gen.su ./grapics/src/misc/lv_templ.cyclo ./grapics/src/misc/lv_templ.d ./grapics/src/misc/lv_templ.o ./grapics/src/misc/lv_templ.su ./grapics/src/misc/lv_timer.cyclo ./grapics/src/misc/lv_timer.d ./grapics/src/misc/lv_timer.o ./grapics/src/misc/lv_timer.su ./grapics/src/misc/lv_tlsf.cyclo ./grapics/src/misc/lv_tlsf.d ./grapics/src/misc/lv_tlsf.o ./grapics/src/misc/lv_tlsf.su ./grapics/src/misc/lv_txt.cyclo ./grapics/src/misc/lv_txt.d ./grapics/src/misc/lv_txt.o ./grapics/src/misc/lv_txt.su ./grapics/src/misc/lv_txt_ap.cyclo ./grapics/src/misc/lv_txt_ap.d ./grapics/src/misc/lv_txt_ap.o ./grapics/src/misc/lv_txt_ap.su ./grapics/src/misc/lv_utils.cyclo ./grapics/src/misc/lv_utils.d ./grapics/src/misc/lv_utils.o ./grapics/src/misc/lv_utils.su

.PHONY: clean-grapics-2f-src-2f-misc

