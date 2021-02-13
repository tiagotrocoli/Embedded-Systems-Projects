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
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
HSE_SYSCLK_8Mhz.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
HSE_SYSCLK_8Mhz.list \

OBJCOPY_BIN += \
HSE_SYSCLK_8Mhz.bin \


# All Target
all: HSE_SYSCLK_8Mhz.elf secondary-outputs

# Tool invocations
HSE_SYSCLK_8Mhz.elf: $(OBJS) $(USER_OBJS) /home/tiago/Documents/MCU_Course/MCU1/HSE_SYSCLK_8Mhz/STM32F407VGTX_FLASH.ld
	arm-none-eabi-gcc -o "HSE_SYSCLK_8Mhz.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/tiago/Documents/MCU_Course/MCU1/HSE_SYSCLK_8Mhz/STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="HSE_SYSCLK_8Mhz.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

HSE_SYSCLK_8Mhz.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "HSE_SYSCLK_8Mhz.list"
	@echo 'Finished building: $@'
	@echo ' '

HSE_SYSCLK_8Mhz.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "HSE_SYSCLK_8Mhz.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
