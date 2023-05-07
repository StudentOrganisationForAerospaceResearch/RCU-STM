################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/RunInterface.cpp 

OBJS += \
./Core/RunInterface.o 

CPP_DEPS += \
./Core/RunInterface.d 


# Each subdirectory must supply rules for building sources it contributes
Core/%.o Core/%.su: ../Core/%.cpp Core/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++20 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L431xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/shiva/Desktop/RCU-STM/Components/_Libraries/embedded-template-library/include" -I"C:/Users/shiva/Desktop/RCU-STM/Components/Communication/Inc" -I"C:/Users/shiva/Desktop/RCU-STM/Components/Core/Inc" -I"C:/Users/shiva/Desktop/RCU-STM/Components/FlightControl/Inc" -I"C:/Users/shiva/Desktop/RCU-STM/Components/SoarDebug/Inc" -I"C:/Users/shiva/Desktop/RCU-STM/Components" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -Wno-volatile -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core

clean-Core:
	-$(RM) ./Core/RunInterface.d ./Core/RunInterface.o ./Core/RunInterface.su

.PHONY: clean-Core

