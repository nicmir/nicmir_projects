################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/algo_suivi_bords.c \
../Core/Src/imu.c \
../Core/Src/main.c \
../Core/Src/parametres_configuration.c \
../Core/Src/pid.c \
../Core/Src/radio_commandes.c \
../Core/Src/shell.c \
../Core/Src/stm32f7xx_hal_msp.c \
../Core/Src/stm32f7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f7xx.c \
../Core/Src/telemetrie.c \
../Core/Src/tfminiplus.c 

OBJS += \
./Core/Src/algo_suivi_bords.o \
./Core/Src/imu.o \
./Core/Src/main.o \
./Core/Src/parametres_configuration.o \
./Core/Src/pid.o \
./Core/Src/radio_commandes.o \
./Core/Src/shell.o \
./Core/Src/stm32f7xx_hal_msp.o \
./Core/Src/stm32f7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f7xx.o \
./Core/Src/telemetrie.o \
./Core/Src/tfminiplus.o 

C_DEPS += \
./Core/Src/algo_suivi_bords.d \
./Core/Src/imu.d \
./Core/Src/main.d \
./Core/Src/parametres_configuration.d \
./Core/Src/pid.d \
./Core/Src/radio_commandes.d \
./Core/Src/shell.d \
./Core/Src/stm32f7xx_hal_msp.d \
./Core/Src/stm32f7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f7xx.d \
./Core/Src/telemetrie.d \
./Core/Src/tfminiplus.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F756xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

