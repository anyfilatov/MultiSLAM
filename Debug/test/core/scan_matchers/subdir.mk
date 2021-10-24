################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../test/core/scan_matchers/bf_multi_res_sm_smoke_test.cpp \
../test/core/scan_matchers/brute_force_sm_smoke_test.cpp \
../test/core/scan_matchers/hill_climbing_sm_smoke_test.cpp \
../test/core/scan_matchers/occupancy_observation_probability_test.cpp 

OBJS += \
./test/core/scan_matchers/bf_multi_res_sm_smoke_test.o \
./test/core/scan_matchers/brute_force_sm_smoke_test.o \
./test/core/scan_matchers/hill_climbing_sm_smoke_test.o \
./test/core/scan_matchers/occupancy_observation_probability_test.o 

CPP_DEPS += \
./test/core/scan_matchers/bf_multi_res_sm_smoke_test.d \
./test/core/scan_matchers/brute_force_sm_smoke_test.d \
./test/core/scan_matchers/hill_climbing_sm_smoke_test.d \
./test/core/scan_matchers/occupancy_observation_probability_test.d 


# Each subdirectory must supply rules for building sources it contributes
test/core/scan_matchers/%.o: ../test/core/scan_matchers/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++1y -I/opt/ros/kinetic/include -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


