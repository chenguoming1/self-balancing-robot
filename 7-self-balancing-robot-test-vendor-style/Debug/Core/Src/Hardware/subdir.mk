################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Hardware/PS2.c \
../Core/Src/Hardware/SR04.c \
../Core/Src/Hardware/adc.c \
../Core/Src/Hardware/control.c \
../Core/Src/Hardware/encoder.c \
../Core/Src/Hardware/key.c \
../Core/Src/Hardware/led.c \
../Core/Src/Hardware/motor.c \
../Core/Src/Hardware/mpu6050.c \
../Core/Src/Hardware/oled.c \
../Core/Src/Hardware/timer.c \
../Core/Src/Hardware/usart2.c 

OBJS += \
./Core/Src/Hardware/PS2.o \
./Core/Src/Hardware/SR04.o \
./Core/Src/Hardware/adc.o \
./Core/Src/Hardware/control.o \
./Core/Src/Hardware/encoder.o \
./Core/Src/Hardware/key.o \
./Core/Src/Hardware/led.o \
./Core/Src/Hardware/motor.o \
./Core/Src/Hardware/mpu6050.o \
./Core/Src/Hardware/oled.o \
./Core/Src/Hardware/timer.o \
./Core/Src/Hardware/usart2.o 

C_DEPS += \
./Core/Src/Hardware/PS2.d \
./Core/Src/Hardware/SR04.d \
./Core/Src/Hardware/adc.d \
./Core/Src/Hardware/control.d \
./Core/Src/Hardware/encoder.d \
./Core/Src/Hardware/key.d \
./Core/Src/Hardware/led.d \
./Core/Src/Hardware/motor.d \
./Core/Src/Hardware/mpu6050.d \
./Core/Src/Hardware/oled.d \
./Core/Src/Hardware/timer.d \
./Core/Src/Hardware/usart2.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Hardware/%.o Core/Src/Hardware/%.su Core/Src/Hardware/%.cyclo: ../Core/Src/Hardware/%.c Core/Src/Hardware/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Core/Inc/Hardware -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Drivers/eMPL -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Hardware

clean-Core-2f-Src-2f-Hardware:
	-$(RM) ./Core/Src/Hardware/PS2.cyclo ./Core/Src/Hardware/PS2.d ./Core/Src/Hardware/PS2.o ./Core/Src/Hardware/PS2.su ./Core/Src/Hardware/SR04.cyclo ./Core/Src/Hardware/SR04.d ./Core/Src/Hardware/SR04.o ./Core/Src/Hardware/SR04.su ./Core/Src/Hardware/adc.cyclo ./Core/Src/Hardware/adc.d ./Core/Src/Hardware/adc.o ./Core/Src/Hardware/adc.su ./Core/Src/Hardware/control.cyclo ./Core/Src/Hardware/control.d ./Core/Src/Hardware/control.o ./Core/Src/Hardware/control.su ./Core/Src/Hardware/encoder.cyclo ./Core/Src/Hardware/encoder.d ./Core/Src/Hardware/encoder.o ./Core/Src/Hardware/encoder.su ./Core/Src/Hardware/key.cyclo ./Core/Src/Hardware/key.d ./Core/Src/Hardware/key.o ./Core/Src/Hardware/key.su ./Core/Src/Hardware/led.cyclo ./Core/Src/Hardware/led.d ./Core/Src/Hardware/led.o ./Core/Src/Hardware/led.su ./Core/Src/Hardware/motor.cyclo ./Core/Src/Hardware/motor.d ./Core/Src/Hardware/motor.o ./Core/Src/Hardware/motor.su ./Core/Src/Hardware/mpu6050.cyclo ./Core/Src/Hardware/mpu6050.d ./Core/Src/Hardware/mpu6050.o ./Core/Src/Hardware/mpu6050.su ./Core/Src/Hardware/oled.cyclo ./Core/Src/Hardware/oled.d ./Core/Src/Hardware/oled.o ./Core/Src/Hardware/oled.su ./Core/Src/Hardware/timer.cyclo ./Core/Src/Hardware/timer.d ./Core/Src/Hardware/timer.o ./Core/Src/Hardware/timer.su ./Core/Src/Hardware/usart2.cyclo ./Core/Src/Hardware/usart2.d ./Core/Src/Hardware/usart2.o ./Core/Src/Hardware/usart2.su

.PHONY: clean-Core-2f-Src-2f-Hardware

