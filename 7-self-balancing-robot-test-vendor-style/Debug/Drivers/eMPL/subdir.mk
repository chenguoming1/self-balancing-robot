################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/eMPL/inv_mpu.c \
../Drivers/eMPL/inv_mpu_dmp_motion_driver.c 

OBJS += \
./Drivers/eMPL/inv_mpu.o \
./Drivers/eMPL/inv_mpu_dmp_motion_driver.o 

C_DEPS += \
./Drivers/eMPL/inv_mpu.d \
./Drivers/eMPL/inv_mpu_dmp_motion_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/eMPL/%.o Drivers/eMPL/%.su Drivers/eMPL/%.cyclo: ../Drivers/eMPL/%.c Drivers/eMPL/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Core/Inc/Hardware -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Drivers/eMPL -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-eMPL

clean-Drivers-2f-eMPL:
	-$(RM) ./Drivers/eMPL/inv_mpu.cyclo ./Drivers/eMPL/inv_mpu.d ./Drivers/eMPL/inv_mpu.o ./Drivers/eMPL/inv_mpu.su ./Drivers/eMPL/inv_mpu_dmp_motion_driver.cyclo ./Drivers/eMPL/inv_mpu_dmp_motion_driver.d ./Drivers/eMPL/inv_mpu_dmp_motion_driver.o ./Drivers/eMPL/inv_mpu_dmp_motion_driver.su

.PHONY: clean-Drivers-2f-eMPL

