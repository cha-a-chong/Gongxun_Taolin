################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Hardware/TX/Tx.c 

OBJS += \
./Hardware/TX/Tx.o 

C_DEPS += \
./Hardware/TX/Tx.d 


# Each subdirectory must supply rules for building sources it contributes
Hardware/TX/%.o Hardware/TX/%.su Hardware/TX/%.cyclo: ../Hardware/TX/%.c Hardware/TX/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"G:/STM32WorkSpace/Gongxun_Taolin/Hardware" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Hardware-2f-TX

clean-Hardware-2f-TX:
	-$(RM) ./Hardware/TX/Tx.cyclo ./Hardware/TX/Tx.d ./Hardware/TX/Tx.o ./Hardware/TX/Tx.su

.PHONY: clean-Hardware-2f-TX

