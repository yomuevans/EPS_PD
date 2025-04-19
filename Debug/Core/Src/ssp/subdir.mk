################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ssp/ssp_comm.c \
../Core/Src/ssp/ssp_command.c \
../Core/Src/ssp/ssp_frame.c \
../Core/Src/ssp/ssp_init.c \
../Core/Src/ssp/ssp_power.c \
../Core/Src/ssp/ssp_telemetry.c 

OBJS += \
./Core/Src/ssp/ssp_comm.o \
./Core/Src/ssp/ssp_command.o \
./Core/Src/ssp/ssp_frame.o \
./Core/Src/ssp/ssp_init.o \
./Core/Src/ssp/ssp_power.o \
./Core/Src/ssp/ssp_telemetry.o 

C_DEPS += \
./Core/Src/ssp/ssp_comm.d \
./Core/Src/ssp/ssp_command.d \
./Core/Src/ssp/ssp_frame.d \
./Core/Src/ssp/ssp_init.d \
./Core/Src/ssp/ssp_power.d \
./Core/Src/ssp/ssp_telemetry.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/ssp/%.o Core/Src/ssp/%.su Core/Src/ssp/%.cyclo: ../Core/Src/ssp/%.c Core/Src/ssp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L496xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-ssp

clean-Core-2f-Src-2f-ssp:
	-$(RM) ./Core/Src/ssp/ssp_comm.cyclo ./Core/Src/ssp/ssp_comm.d ./Core/Src/ssp/ssp_comm.o ./Core/Src/ssp/ssp_comm.su ./Core/Src/ssp/ssp_command.cyclo ./Core/Src/ssp/ssp_command.d ./Core/Src/ssp/ssp_command.o ./Core/Src/ssp/ssp_command.su ./Core/Src/ssp/ssp_frame.cyclo ./Core/Src/ssp/ssp_frame.d ./Core/Src/ssp/ssp_frame.o ./Core/Src/ssp/ssp_frame.su ./Core/Src/ssp/ssp_init.cyclo ./Core/Src/ssp/ssp_init.d ./Core/Src/ssp/ssp_init.o ./Core/Src/ssp/ssp_init.su ./Core/Src/ssp/ssp_power.cyclo ./Core/Src/ssp/ssp_power.d ./Core/Src/ssp/ssp_power.o ./Core/Src/ssp/ssp_power.su ./Core/Src/ssp/ssp_telemetry.cyclo ./Core/Src/ssp/ssp_telemetry.d ./Core/Src/ssp/ssp_telemetry.o ./Core/Src/ssp/ssp_telemetry.su

.PHONY: clean-Core-2f-Src-2f-ssp

