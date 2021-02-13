################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
User_app_STM32F4xx.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
User_app_STM32F4xx.list \

OBJCOPY_HEX += \
User_app_STM32F4xx.hex \

OBJCOPY_BIN += \
User_app_STM32F4xx.bin \


# All Target
all: User_app_STM32F4xx.elf secondary-outputs

# Tool invocations
User_app_STM32F4xx.elf: $(OBJS) $(USER_OBJS) C:\Users\Tiago\Documents\MCU_Course\MCU1\User_app_STM32F4xx\STM32F407VGTX_FLASH.ld
	arm-none-eabi-gcc -o "User_app_STM32F4xx.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Tiago\Documents\MCU_Course\MCU1\User_app_STM32F4xx\STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="User_app_STM32F4xx.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

User_app_STM32F4xx.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "User_app_STM32F4xx.list"
	@echo 'Finished building: $@'
	@echo ' '

User_app_STM32F4xx.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "User_app_STM32F4xx.hex"
	@echo 'Finished building: $@'
	@echo ' '

User_app_STM32F4xx.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "User_app_STM32F4xx.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
