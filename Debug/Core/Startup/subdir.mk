################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32h743iikx.s 

OBJS += \
./Core/Startup/startup_stm32h743iikx.o 

S_DEPS += \
./Core/Startup/startup_stm32h743iikx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/startup_stm32h743iikx.o: ../Core/Startup/startup_stm32h743iikx.s
	arm-none-eabi-gcc -mcpu=cortex-m7 -g3 -c -x assembler-with-cpp -MMD -MP -MF"Core/Startup/startup_stm32h743iikx.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

