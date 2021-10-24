################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/slams/gmapping/gmapping.cpp 

OBJS += \
./src/slams/gmapping/gmapping.o 

CPP_DEPS += \
./src/slams/gmapping/gmapping.d 


# Each subdirectory must supply rules for building sources it contributes
src/slams/gmapping/%.o: ../src/slams/gmapping/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++1y -I/opt/ros/kinetic/include -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


