################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FreeRTOS/Source/portable/MemMang/heap_4.c 

OBJS += \
./FreeRTOS/Source/portable/MemMang/heap_4.o 

C_DEPS += \
./FreeRTOS/Source/portable/MemMang/heap_4.d 


# Each subdirectory must supply rules for building sources it contributes
FreeRTOS/Source/portable/MemMang/%.o FreeRTOS/Source/portable/MemMang/%.su FreeRTOS/Source/portable/MemMang/%.cyclo: ../FreeRTOS/Source/portable/MemMang/%.c FreeRTOS/Source/portable/MemMang/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"C:/Users/mihap/STM32CubeIDE/workspace_1.16.1/PRIM-FreeRTOS/FreeRTOS/Source/include" -I"C:/Users/mihap/STM32CubeIDE/workspace_1.16.1/PRIM-FreeRTOS/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/mihap/STM32CubeIDE/workspace_1.16.1/PRIM-FreeRTOS/FreeRTOS/Source" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-FreeRTOS-2f-Source-2f-portable-2f-MemMang

clean-FreeRTOS-2f-Source-2f-portable-2f-MemMang:
	-$(RM) ./FreeRTOS/Source/portable/MemMang/heap_4.cyclo ./FreeRTOS/Source/portable/MemMang/heap_4.d ./FreeRTOS/Source/portable/MemMang/heap_4.o ./FreeRTOS/Source/portable/MemMang/heap_4.su

.PHONY: clean-FreeRTOS-2f-Source-2f-portable-2f-MemMang

