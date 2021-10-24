################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include test/utils/data_generation/subdir.mk
-include test/core/states/subdir.mk
-include test/core/scan_matchers/subdir.mk
-include test/core/maps/subdir.mk
-include test/core/features/subdir.mk
-include test/core/subdir.mk
-include src/utils/subdir.mk
-include src/slams/vinyx/subdir.mk
-include src/slams/viny/subdir.mk
-include src/slams/tiny/subdir.mk
-include src/slams/gmapping/subdir.mk
-include src/slams/credibilist/subdir.mk
-include src/ros/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: TBM-merger

# Tool invocations
TBM-merger: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross G++ Linker'
	g++  -o "TBM-merger" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(CC_DEPS)$(C++_DEPS)$(EXECUTABLES)$(OBJS)$(C_UPPER_DEPS)$(CXX_DEPS)$(C_DEPS)$(CPP_DEPS) TBM-merger
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
