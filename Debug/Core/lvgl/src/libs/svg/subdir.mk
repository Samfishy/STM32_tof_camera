################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lvgl/src/libs/svg/lv_svg.c \
../Core/lvgl/src/libs/svg/lv_svg_decoder.c \
../Core/lvgl/src/libs/svg/lv_svg_parser.c \
../Core/lvgl/src/libs/svg/lv_svg_render.c \
../Core/lvgl/src/libs/svg/lv_svg_token.c 

OBJS += \
./Core/lvgl/src/libs/svg/lv_svg.o \
./Core/lvgl/src/libs/svg/lv_svg_decoder.o \
./Core/lvgl/src/libs/svg/lv_svg_parser.o \
./Core/lvgl/src/libs/svg/lv_svg_render.o \
./Core/lvgl/src/libs/svg/lv_svg_token.o 

C_DEPS += \
./Core/lvgl/src/libs/svg/lv_svg.d \
./Core/lvgl/src/libs/svg/lv_svg_decoder.d \
./Core/lvgl/src/libs/svg/lv_svg_parser.d \
./Core/lvgl/src/libs/svg/lv_svg_render.d \
./Core/lvgl/src/libs/svg/lv_svg_token.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lvgl/src/libs/svg/%.o Core/lvgl/src/libs/svg/%.su Core/lvgl/src/libs/svg/%.cyclo: ../Core/lvgl/src/libs/svg/%.c Core/lvgl/src/libs/svg/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/lvgl" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lvgl-2f-src-2f-libs-2f-svg

clean-Core-2f-lvgl-2f-src-2f-libs-2f-svg:
	-$(RM) ./Core/lvgl/src/libs/svg/lv_svg.cyclo ./Core/lvgl/src/libs/svg/lv_svg.d ./Core/lvgl/src/libs/svg/lv_svg.o ./Core/lvgl/src/libs/svg/lv_svg.su ./Core/lvgl/src/libs/svg/lv_svg_decoder.cyclo ./Core/lvgl/src/libs/svg/lv_svg_decoder.d ./Core/lvgl/src/libs/svg/lv_svg_decoder.o ./Core/lvgl/src/libs/svg/lv_svg_decoder.su ./Core/lvgl/src/libs/svg/lv_svg_parser.cyclo ./Core/lvgl/src/libs/svg/lv_svg_parser.d ./Core/lvgl/src/libs/svg/lv_svg_parser.o ./Core/lvgl/src/libs/svg/lv_svg_parser.su ./Core/lvgl/src/libs/svg/lv_svg_render.cyclo ./Core/lvgl/src/libs/svg/lv_svg_render.d ./Core/lvgl/src/libs/svg/lv_svg_render.o ./Core/lvgl/src/libs/svg/lv_svg_render.su ./Core/lvgl/src/libs/svg/lv_svg_token.cyclo ./Core/lvgl/src/libs/svg/lv_svg_token.d ./Core/lvgl/src/libs/svg/lv_svg_token.o ./Core/lvgl/src/libs/svg/lv_svg_token.su

.PHONY: clean-Core-2f-lvgl-2f-src-2f-libs-2f-svg

