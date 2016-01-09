################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Thread/ThImuRead.cpp \
../Thread/ThMission.cpp \
../Thread/ThSolar.cpp \
../Thread/ThTelecommand.cpp \
../Thread/ThTelemetry.cpp 

OBJS += \
./Thread/ThImuRead.o \
./Thread/ThMission.o \
./Thread/ThSolar.o \
./Thread/ThTelecommand.o \
./Thread/ThTelemetry.o 

CPP_DEPS += \
./Thread/ThImuRead.d \
./Thread/ThMission.d \
./Thread/ThSolar.d \
./Thread/ThTelecommand.d \
./Thread/ThTelemetry.d 


# Each subdirectory must supply rules for building sources it contributes
Thread/%.o: ../Thread/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -DUSE_STM32_DISCOVERY -DSTM32F40_41xxx -I"C:\Users\pinker\Desktop\Floatsat\Eclipse Luna SR1 121\Workspace\rodos\src\bare-metal\stm32f4\STM32F4xx_StdPeriph_Driver\inc" -I"C:\Users\pinker\Desktop\Floatsat\Eclipse Luna SR1 121\Workspace\rodos\src\bare-metal\stm32f4\CMSIS\Device\ST\STM32F4xx\Include" -I"C:\Users\pinker\Desktop\Floatsat\Eclipse Luna SR1 121\Workspace\rodos\src\bare-metal\stm32f4\CMSIS\Include" -I"C:\Users\pinker\Desktop\Floatsat\Eclipse Luna SR1 121\Workspace\rodos\src\bare-metal\stm32f4\hal" -I"C:\Users\pinker\Desktop\Floatsat\Eclipse Luna SR1 121\Workspace\rodos\src\bare-metal\stm32f4" -I"C:\Users\pinker\Desktop\Floatsat\Eclipse Luna SR1 121\Workspace\rodos\src\bare-metal-generic" -I"C:\Users\pinker\Desktop\Floatsat\Eclipse Luna SR1 121\Workspace\rodos\src\independent\gateway" -I"C:\Users\pinker\Desktop\Floatsat\Eclipse Luna SR1 121\Workspace\rodos\src\independent" -I"C:\Users\pinker\Desktop\Floatsat\Eclipse Luna SR1 121\Workspace\rodos\api" -I"C:\Users\pinker\Desktop\Floatsat\Eclipse Luna SR1 121\Workspace\rodos\src\bare-metal\stm32f4\sdCard" -I"C:\Users\pinker\Desktop\Floatsat\Eclipse Luna SR1 121\Workspace\rodos\api\hal" -I"C:\Users\pinker\Desktop\Floatsat\Eclipse Luna SR1 121\Workspace\rodos\default_usr_configs" -I"C:\Users\pinker\Desktop\Floatsat\Eclipse Luna SR1 121\Workspace\support_libs" -I"C:\Users\pinker\Desktop\Floatsat\Eclipse Luna SR1 121\Workspace\support_libs\flash\spiFlash_AT45DBxxx" -fabi-version=0 -fno-exceptions -fno-rtti -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


