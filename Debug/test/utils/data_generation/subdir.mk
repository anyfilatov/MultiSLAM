################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../test/utils/data_generation/grid_map_patcher_test.cpp \
../test/utils/data_generation/laser_scan_generator_test.cpp \
../test/utils/data_generation/map_primitives_test.cpp 

OBJS += \
./test/utils/data_generation/grid_map_patcher_test.o \
./test/utils/data_generation/laser_scan_generator_test.o \
./test/utils/data_generation/map_primitives_test.o 

CPP_DEPS += \
./test/utils/data_generation/grid_map_patcher_test.d \
./test/utils/data_generation/laser_scan_generator_test.d \
./test/utils/data_generation/map_primitives_test.d 


# Each subdirectory must supply rules for building sources it contributes
test/utils/data_generation/%.o: ../test/utils/data_generation/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++1y -I/opt/ros/kinetic/include -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


