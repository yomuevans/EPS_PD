################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/app/Log.c \
../Core/Src/app/delay.c \
../Core/Src/app/eeprom.c \
../Core/Src/app/fault.c \
../Core/Src/app/i2c_comm.c \
../Core/Src/app/satellite_modes.c \
../Core/Src/app/ssp.c \
../Core/Src/app/sync_counter.c \
../Core/Src/app/telemetry.c 

OBJS += \
./Core/Src/app/Log.o \
./Core/Src/app/delay.o \
./Core/Src/app/eeprom.o \
./Core/Src/app/fault.o \
./Core/Src/app/i2c_comm.o \
./Core/Src/app/satellite_modes.o \
./Core/Src/app/ssp.o \
./Core/Src/app/sync_counter.o \
./Core/Src/app/telemetry.o 

C_DEPS += \
./Core/Src/app/Log.d \
./Core/Src/app/delay.d \
./Core/Src/app/eeprom.d \
./Core/Src/app/fault.d \
./Core/Src/app/i2c_comm.d \
./Core/Src/app/satellite_modes.d \
./Core/Src/app/ssp.d \
./Core/Src/app/sync_counter.d \
./Core/Src/app/telemetry.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/app/%.o Core/Src/app/%.su Core/Src/app/%.cyclo: ../Core/Src/app/%.c Core/Src/app/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L496xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-app

clean-Core-2f-Src-2f-app:
	-$(RM) ./Core/Src/app/Log.cyclo ./Core/Src/app/Log.d ./Core/Src/app/Log.o ./Core/Src/app/Log.su ./Core/Src/app/delay.cyclo ./Core/Src/app/delay.d ./Core/Src/app/delay.o ./Core/Src/app/delay.su ./Core/Src/app/eeprom.cyclo ./Core/Src/app/eeprom.d ./Core/Src/app/eeprom.o ./Core/Src/app/eeprom.su ./Core/Src/app/fault.cyclo ./Core/Src/app/fault.d ./Core/Src/app/fault.o ./Core/Src/app/fault.su ./Core/Src/app/i2c_comm.cyclo ./Core/Src/app/i2c_comm.d ./Core/Src/app/i2c_comm.o ./Core/Src/app/i2c_comm.su ./Core/Src/app/satellite_modes.cyclo ./Core/Src/app/satellite_modes.d ./Core/Src/app/satellite_modes.o ./Core/Src/app/satellite_modes.su ./Core/Src/app/ssp.cyclo ./Core/Src/app/ssp.d ./Core/Src/app/ssp.o ./Core/Src/app/ssp.su ./Core/Src/app/sync_counter.cyclo ./Core/Src/app/sync_counter.d ./Core/Src/app/sync_counter.o ./Core/Src/app/sync_counter.su ./Core/Src/app/telemetry.cyclo ./Core/Src/app/telemetry.d ./Core/Src/app/telemetry.o ./Core/Src/app/telemetry.su

.PHONY: clean-Core-2f-Src-2f-app

