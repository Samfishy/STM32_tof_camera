################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../grapics/demos/music/lv_demo_music.c \
../grapics/demos/music/lv_demo_music_list.c \
../grapics/demos/music/lv_demo_music_main.c 

OBJS += \
./grapics/demos/music/lv_demo_music.o \
./grapics/demos/music/lv_demo_music_list.o \
./grapics/demos/music/lv_demo_music_main.o 

C_DEPS += \
./grapics/demos/music/lv_demo_music.d \
./grapics/demos/music/lv_demo_music_list.d \
./grapics/demos/music/lv_demo_music_main.d 


# Each subdirectory must supply rules for building sources it contributes
grapics/demos/music/%.o grapics/demos/music/%.su grapics/demos/music/%.cyclo: ../grapics/demos/music/%.c grapics/demos/music/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-grapics-2f-demos-2f-music

clean-grapics-2f-demos-2f-music:
	-$(RM) ./grapics/demos/music/lv_demo_music.cyclo ./grapics/demos/music/lv_demo_music.d ./grapics/demos/music/lv_demo_music.o ./grapics/demos/music/lv_demo_music.su ./grapics/demos/music/lv_demo_music_list.cyclo ./grapics/demos/music/lv_demo_music_list.d ./grapics/demos/music/lv_demo_music_list.o ./grapics/demos/music/lv_demo_music_list.su ./grapics/demos/music/lv_demo_music_main.cyclo ./grapics/demos/music/lv_demo_music_main.d ./grapics/demos/music/lv_demo_music_main.o ./grapics/demos/music/lv_demo_music_main.su

.PHONY: clean-grapics-2f-demos-2f-music

