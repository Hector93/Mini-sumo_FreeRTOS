# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

# compile ASM with arm-none-eabi-gcc
# compile C with arm-none-eabi-gcc
ASM_FLAGS = -x assembler-with-cpp -mcpu=cortex-m3 -mthumb   -Wall -Wextra -Wdouble-promotion -fmessage-length=0 -DUSE_STDPERIPH_DRIVER -DEMPL -DUSE_DMP -DMPU6050 -DMPL_LOG_NDEBUG=1 -DEMPL_TARGET_STM32F4 -DREMOVE_LOGGING -DPRINT_IMU_DATA -DPRINT_IMU_QUAT -DSTM32F1 -DSSD1306_USE_I2C -I"Middlewares/Third_Party/InvenSense/core/driver/include" -I"Middlewares/Third_Party/InvenSense/core/driver/eMPL" -I"Middlewares/Third_Party/InvenSense/core/driver/stm32L" -I"Middlewares/Third_Party/InvenSense/core/mpl" -I"Middlewares/Third_Party/InvenSense/core/mllite" -I"Middlewares/Third_Party/InvenSense/core/mpl" -Os -Wall -fdata-sections -ffunction-sections -dM -fno-common -fshort-enums -g   -DUSE_HAL_DRIVER -DSTM32F103xB

ASM_DEFINES = -DVERSION=\"2006010150\"

ASM_INCLUDES = -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Application/Inc -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Drivers/CMSIS/Device/ST/STM32F1xx/Include -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Drivers/CMSIS/Include -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Drivers/STM32F1xx_HAL_Driver/Inc -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Inc -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Middlewares/Third_Party/FreeRTOS/Source/include -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Middlewares/Third_Party/InvenSense/core/driver/eMPL -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Middlewares/Third_Party/InvenSense/core/driver/include -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Middlewares/Third_Party/InvenSense/core/driver/stm32L -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Middlewares/Third_Party/InvenSense/core/eMPL-hal -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Middlewares/Third_Party/InvenSense/core/mllite -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Middlewares/Third_Party/InvenSense/core/mpl -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Middlewares/Third_Party/InvenSense -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Middlewares/Third_Party/eFLL -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Middlewares/Third_Party/stm32-ssd1306/ssd1306 

C_FLAGS = -mcpu=cortex-m3 -mthumb   -Wall -Wextra -Wdouble-promotion -fmessage-length=0 -DUSE_STDPERIPH_DRIVER -DEMPL -DUSE_DMP -DMPU6050 -DMPL_LOG_NDEBUG=1 -DEMPL_TARGET_STM32F4 -DREMOVE_LOGGING -DPRINT_IMU_DATA -DPRINT_IMU_QUAT -DSTM32F1 -DSSD1306_USE_I2C -I"Middlewares/Third_Party/InvenSense/core/driver/include" -I"Middlewares/Third_Party/InvenSense/core/driver/eMPL" -I"Middlewares/Third_Party/InvenSense/core/driver/stm32L" -I"Middlewares/Third_Party/InvenSense/core/mpl" -I"Middlewares/Third_Party/InvenSense/core/mllite" -I"Middlewares/Third_Party/InvenSense/core/mpl" -Os -Wall -fdata-sections -ffunction-sections -dM -fno-common -fshort-enums  -DDEBUG -g -gdwarf-2   -DUSE_HAL_DRIVER -DSTM32F103xB

C_DEFINES = -DVERSION=\"2006010150\"

C_INCLUDES = -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Application/Inc -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Drivers/CMSIS/Device/ST/STM32F1xx/Include -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Drivers/CMSIS/Include -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Drivers/STM32F1xx_HAL_Driver/Inc -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Inc -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Middlewares/Third_Party/FreeRTOS/Source/include -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Middlewares/Third_Party/InvenSense/core/driver/eMPL -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Middlewares/Third_Party/InvenSense/core/driver/include -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Middlewares/Third_Party/InvenSense/core/driver/stm32L -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Middlewares/Third_Party/InvenSense/core/eMPL-hal -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Middlewares/Third_Party/InvenSense/core/mllite -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Middlewares/Third_Party/InvenSense/core/mpl -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Middlewares/Third_Party/InvenSense -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Middlewares/Third_Party/eFLL -I/home/hector/Documents/Robotica/firmware/borrar/minisumo128k/Middlewares/Third_Party/stm32-ssd1306/ssd1306 

