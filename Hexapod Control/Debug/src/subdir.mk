################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Accelerometer.cpp \
../src/CommsManager.cpp \
../src/Hexapod.cpp \
../src/Leg.cpp \
../src/Motor.cpp \
../src/PID.cpp \
../src/Path.cpp \
../src/Remote.cpp \
../src/main.cpp 

OBJS += \
./src/Accelerometer.o \
./src/CommsManager.o \
./src/Hexapod.o \
./src/Leg.o \
./src/Motor.o \
./src/PID.o \
./src/Path.o \
./src/Remote.o \
./src/main.o 

CPP_DEPS += \
./src/Accelerometer.d \
./src/CommsManager.d \
./src/Hexapod.d \
./src/Leg.d \
./src/Motor.d \
./src/PID.d \
./src/Path.d \
./src/Remote.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f4-hal" -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


