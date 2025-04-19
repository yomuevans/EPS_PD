################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/eps_faults/eps_faults.c 

OBJS += \
./Core/Src/eps_faults/eps_faults.o 

C_DEPS += \
./Core/Src/eps_faults/eps_faults.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/eps_faults/%.o Core/Src/eps_faults/%.su Core/Src/eps_faults/%.cyclo: ../Core/Src/eps_faults/%.c Core/Src/eps_faults/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L496xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-eps_faults

clean-Core-2f-Src-2f-eps_faults:
	-$(RM) ./Core/Src/eps_faults/eps_faults.cyclo ./Core/Src/eps_faults/eps_faults.d ./Core/Src/eps_faults/eps_faults.o ./Core/Src/eps_faults/eps_faults.su

.PHONY: clean-Core-2f-Src-2f-eps_faults

