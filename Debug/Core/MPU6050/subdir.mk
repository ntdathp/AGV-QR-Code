################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/MPU6050/mpu6050.c 

OBJS += \
./Core/MPU6050/mpu6050.o 

C_DEPS += \
./Core/MPU6050/mpu6050.d 


# Each subdirectory must supply rules for building sources it contributes
Core/MPU6050/%.o Core/MPU6050/%.su Core/MPU6050/%.cyclo: ../Core/MPU6050/%.c Core/MPU6050/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-MPU6050

clean-Core-2f-MPU6050:
	-$(RM) ./Core/MPU6050/mpu6050.cyclo ./Core/MPU6050/mpu6050.d ./Core/MPU6050/mpu6050.o ./Core/MPU6050/mpu6050.su

.PHONY: clean-Core-2f-MPU6050

