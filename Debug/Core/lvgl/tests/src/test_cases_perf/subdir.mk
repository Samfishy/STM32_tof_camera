################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lvgl/tests/src/test_cases_perf/test_chart.c \
../Core/lvgl/tests/src/test_cases_perf/test_label.c \
../Core/lvgl/tests/src/test_cases_perf/test_math.c 

OBJS += \
./Core/lvgl/tests/src/test_cases_perf/test_chart.o \
./Core/lvgl/tests/src/test_cases_perf/test_label.o \
./Core/lvgl/tests/src/test_cases_perf/test_math.o 

C_DEPS += \
./Core/lvgl/tests/src/test_cases_perf/test_chart.d \
./Core/lvgl/tests/src/test_cases_perf/test_label.d \
./Core/lvgl/tests/src/test_cases_perf/test_math.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lvgl/tests/src/test_cases_perf/%.o Core/lvgl/tests/src/test_cases_perf/%.su Core/lvgl/tests/src/test_cases_perf/%.cyclo: ../Core/lvgl/tests/src/test_cases_perf/%.c Core/lvgl/tests/src/test_cases_perf/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/lvgl" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/grapics/ui" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/Include" -I"/home/samfishy/Documents/STM_Workspace/TFT_Display_F411/Drivers/CMSIS/CMSIS_DSP/PrivateInclude" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lvgl-2f-tests-2f-src-2f-test_cases_perf

clean-Core-2f-lvgl-2f-tests-2f-src-2f-test_cases_perf:
	-$(RM) ./Core/lvgl/tests/src/test_cases_perf/test_chart.cyclo ./Core/lvgl/tests/src/test_cases_perf/test_chart.d ./Core/lvgl/tests/src/test_cases_perf/test_chart.o ./Core/lvgl/tests/src/test_cases_perf/test_chart.su ./Core/lvgl/tests/src/test_cases_perf/test_label.cyclo ./Core/lvgl/tests/src/test_cases_perf/test_label.d ./Core/lvgl/tests/src/test_cases_perf/test_label.o ./Core/lvgl/tests/src/test_cases_perf/test_label.su ./Core/lvgl/tests/src/test_cases_perf/test_math.cyclo ./Core/lvgl/tests/src/test_cases_perf/test_math.d ./Core/lvgl/tests/src/test_cases_perf/test_math.o ./Core/lvgl/tests/src/test_cases_perf/test_math.su

.PHONY: clean-Core-2f-lvgl-2f-tests-2f-src-2f-test_cases_perf

