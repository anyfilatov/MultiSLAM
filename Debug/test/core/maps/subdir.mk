################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../test/core/maps/area_occupancy_estimator_test.cpp \
../test/core/maps/grid_map_scan_adders_test.cpp \
../test/core/maps/grid_rasterization_test.cpp \
../test/core/maps/regular_squares_grid_test.cpp \
../test/core/maps/rescalable_caching_grid_map_test.cpp \
../test/core/maps/unbounded_lazy_tiled_grid_map_test.cpp \
../test/core/maps/unbounded_plain_grid_map_test.cpp 

OBJS += \
./test/core/maps/area_occupancy_estimator_test.o \
./test/core/maps/grid_map_scan_adders_test.o \
./test/core/maps/grid_rasterization_test.o \
./test/core/maps/regular_squares_grid_test.o \
./test/core/maps/rescalable_caching_grid_map_test.o \
./test/core/maps/unbounded_lazy_tiled_grid_map_test.o \
./test/core/maps/unbounded_plain_grid_map_test.o 

CPP_DEPS += \
./test/core/maps/area_occupancy_estimator_test.d \
./test/core/maps/grid_map_scan_adders_test.d \
./test/core/maps/grid_rasterization_test.d \
./test/core/maps/regular_squares_grid_test.d \
./test/core/maps/rescalable_caching_grid_map_test.d \
./test/core/maps/unbounded_lazy_tiled_grid_map_test.d \
./test/core/maps/unbounded_plain_grid_map_test.d 


# Each subdirectory must supply rules for building sources it contributes
test/core/maps/%.o: ../test/core/maps/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++1y -I/opt/ros/kinetic/include -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


