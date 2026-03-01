################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc.c \
../Core/Src/ads7041.c \
../Core/Src/ads8325.c \
../Core/Src/adxl345.c \
../Core/Src/app_freertos.c \
../Core/Src/dac.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/main.c \
../Core/Src/my_tasks.c \
../Core/Src/spi.c \
../Core/Src/stm32h5xx_hal_msp.c \
../Core/Src/stm32h5xx_it.c \
../Core/Src/switch.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h5xx.c \
../Core/Src/tim.c \
../Core/Src/tle9201.c \
../Core/Src/uart_cmd.c \
../Core/Src/usart.c \
../Core/Src/vca_control.c \
../Core/Src/vca_stub.c 

OBJS += \
./Core/Src/adc.o \
./Core/Src/ads7041.o \
./Core/Src/ads8325.o \
./Core/Src/adxl345.o \
./Core/Src/app_freertos.o \
./Core/Src/dac.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/main.o \
./Core/Src/my_tasks.o \
./Core/Src/spi.o \
./Core/Src/stm32h5xx_hal_msp.o \
./Core/Src/stm32h5xx_it.o \
./Core/Src/switch.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h5xx.o \
./Core/Src/tim.o \
./Core/Src/tle9201.o \
./Core/Src/uart_cmd.o \
./Core/Src/usart.o \
./Core/Src/vca_control.o \
./Core/Src/vca_stub.o 

C_DEPS += \
./Core/Src/adc.d \
./Core/Src/ads7041.d \
./Core/Src/ads8325.d \
./Core/Src/adxl345.d \
./Core/Src/app_freertos.d \
./Core/Src/dac.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/main.d \
./Core/Src/my_tasks.d \
./Core/Src/spi.d \
./Core/Src/stm32h5xx_hal_msp.d \
./Core/Src/stm32h5xx_it.d \
./Core/Src/switch.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h5xx.d \
./Core/Src/tim.d \
./Core/Src/tle9201.d \
./Core/Src/uart_cmd.d \
./Core/Src/usart.d \
./Core/Src/vca_control.d \
./Core/Src/vca_stub.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H562xx -c -I../Core/Inc -I../Drivers/STM32H5xx_HAL_Driver/Inc -I../Drivers/STM32H5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H5xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include/ -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure/ -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/ -I../Middlewares/Third_Party/CMSIS/RTOS2/Include/ -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/ads7041.cyclo ./Core/Src/ads7041.d ./Core/Src/ads7041.o ./Core/Src/ads7041.su ./Core/Src/ads8325.cyclo ./Core/Src/ads8325.d ./Core/Src/ads8325.o ./Core/Src/ads8325.su ./Core/Src/adxl345.cyclo ./Core/Src/adxl345.d ./Core/Src/adxl345.o ./Core/Src/adxl345.su ./Core/Src/app_freertos.cyclo ./Core/Src/app_freertos.d ./Core/Src/app_freertos.o ./Core/Src/app_freertos.su ./Core/Src/dac.cyclo ./Core/Src/dac.d ./Core/Src/dac.o ./Core/Src/dac.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/my_tasks.cyclo ./Core/Src/my_tasks.d ./Core/Src/my_tasks.o ./Core/Src/my_tasks.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32h5xx_hal_msp.cyclo ./Core/Src/stm32h5xx_hal_msp.d ./Core/Src/stm32h5xx_hal_msp.o ./Core/Src/stm32h5xx_hal_msp.su ./Core/Src/stm32h5xx_it.cyclo ./Core/Src/stm32h5xx_it.d ./Core/Src/stm32h5xx_it.o ./Core/Src/stm32h5xx_it.su ./Core/Src/switch.cyclo ./Core/Src/switch.d ./Core/Src/switch.o ./Core/Src/switch.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h5xx.cyclo ./Core/Src/system_stm32h5xx.d ./Core/Src/system_stm32h5xx.o ./Core/Src/system_stm32h5xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/tle9201.cyclo ./Core/Src/tle9201.d ./Core/Src/tle9201.o ./Core/Src/tle9201.su ./Core/Src/uart_cmd.cyclo ./Core/Src/uart_cmd.d ./Core/Src/uart_cmd.o ./Core/Src/uart_cmd.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su ./Core/Src/vca_control.cyclo ./Core/Src/vca_control.d ./Core/Src/vca_control.o ./Core/Src/vca_control.su ./Core/Src/vca_stub.cyclo ./Core/Src/vca_stub.d ./Core/Src/vca_stub.o ./Core/Src/vca_stub.su

.PHONY: clean-Core-2f-Src

