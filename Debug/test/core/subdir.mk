################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../test/core/geometry_discrete_primitives_test.cpp \
../test/core/light_weight_rectangle_test.cpp \
../test/core/trigonometry_utils_test.cpp 

OBJS += \
./test/core/geometry_discrete_primitives_test.o \
./test/core/light_weight_rectangle_test.o \
./test/core/trigonometry_utils_test.o 

CPP_DEPS += \
./test/core/geometry_discrete_primitives_test.d \
./test/core/light_weight_rectangle_test.d \
./test/core/trigonometry_utils_test.d 


# Each subdirectory must supply rules for building sources it contributes
test/core/%.o: ../test/core/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++1y -I/opt/ros/kinetic/include -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


