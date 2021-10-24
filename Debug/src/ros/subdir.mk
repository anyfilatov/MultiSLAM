################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/ros/lslam2D_bag_runner.cpp \
../src/ros/path_publisher.cpp \
../src/ros/wg_pr2_bag_adapter.cpp 

OBJS += \
./src/ros/lslam2D_bag_runner.o \
./src/ros/path_publisher.o \
./src/ros/wg_pr2_bag_adapter.o 

CPP_DEPS += \
./src/ros/lslam2D_bag_runner.d \
./src/ros/path_publisher.d \
./src/ros/wg_pr2_bag_adapter.d 


# Each subdirectory must supply rules for building sources it contributes
src/ros/%.o: ../src/ros/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++1y -I/opt/ros/kinetic/include -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


