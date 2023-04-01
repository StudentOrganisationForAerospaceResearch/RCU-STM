################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Components/Core/Command.cpp \
../Components/Core/Mutex.cpp \
../Components/Core/Queue.cpp \
../Components/Core/Task.cpp 

OBJS += \
./Components/Core/Command.o \
./Components/Core/Mutex.o \
./Components/Core/Queue.o \
./Components/Core/Task.o 

CPP_DEPS += \
./Components/Core/Command.d \
./Components/Core/Mutex.d \
./Components/Core/Queue.d \
./Components/Core/Task.d 


# Each subdirectory must supply rules for building sources it contributes
Components/Core/%.o Components/Core/%.su: ../Components/Core/%.cpp Components/Core/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++20 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L431xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/cjcha/OneDrive/Desktop/Git/RCU-STM/Components/_Libraries/embedded-template-library/include" -I"C:/Users/cjcha/OneDrive/Desktop/Git/RCU-STM/Components/Communication/Inc" -I"C:/Users/cjcha/OneDrive/Desktop/Git/RCU-STM/Components/Core/Inc" -I"C:/Users/cjcha/OneDrive/Desktop/Git/RCU-STM/Components/FlightControl/Inc" -I"C:/Users/cjcha/OneDrive/Desktop/Git/RCU-STM/Components/SoarDebug/Inc" -I"C:/Users/cjcha/OneDrive/Desktop/Git/RCU-STM/Components" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -Wno-volatile -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Components-2f-Core

clean-Components-2f-Core:
	-$(RM) ./Components/Core/Command.d ./Components/Core/Command.o ./Components/Core/Command.su ./Components/Core/Mutex.d ./Components/Core/Mutex.o ./Components/Core/Mutex.su ./Components/Core/Queue.d ./Components/Core/Queue.o ./Components/Core/Queue.su ./Components/Core/Task.d ./Components/Core/Task.o ./Components/Core/Task.su

.PHONY: clean-Components-2f-Core

