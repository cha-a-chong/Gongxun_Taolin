################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Hardware/Time/Time.c 

OBJS += \
./Hardware/Time/Time.o 

C_DEPS += \
./Hardware/Time/Time.d 


# Each subdirectory must supply rules for building sources it contributes
Hardware/Time/%.o Hardware/Time/%.su Hardware/Time/%.cyclo: ../Hardware/Time/%.c Hardware/Time/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"G:/STM32WorkSpace/Gongxun_Taolin/Hardware" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Hardware-2f-Time

clean-Hardware-2f-Time:
	-$(RM) ./Hardware/Time/Time.cyclo ./Hardware/Time/Time.d ./Hardware/Time/Time.o ./Hardware/Time/Time.su

.PHONY: clean-Hardware-2f-Time
