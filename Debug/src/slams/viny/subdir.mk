################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/slams/viny/main.cpp \
../src/slams/viny/viny_slam.cpp 

OBJS += \
./src/slams/viny/main.o \
./src/slams/viny/viny_slam.o 

CPP_DEPS += \
./src/slams/viny/main.d \
./src/slams/viny/viny_slam.d 


# Each subdirectory must supply rules for building sources it contributes
src/slams/viny/%.o: ../src/slams/viny/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++1y -I/opt/ros/kinetic/include -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


