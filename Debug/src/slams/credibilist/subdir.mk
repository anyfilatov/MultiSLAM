################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/slams/credibilist/slam.cpp 

OBJS += \
./src/slams/credibilist/slam.o 

CPP_DEPS += \
./src/slams/credibilist/slam.d 


# Each subdirectory must supply rules for building sources it contributes
src/slams/credibilist/%.o: ../src/slams/credibilist/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++1y -I/opt/ros/kinetic/include -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


