################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../test/core/states/sensor_data_test.cpp 

OBJS += \
./test/core/states/sensor_data_test.o 

CPP_DEPS += \
./test/core/states/sensor_data_test.d 


# Each subdirectory must supply rules for building sources it contributes
test/core/states/%.o: ../test/core/states/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++1y -I/opt/ros/kinetic/include -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


