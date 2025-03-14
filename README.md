# Trinity_FW_SensorProcessor_ZH10-F
Sensor Processor ZH10-F (dust module)

Firmware for sensor processor with ZH10-F module

## Hardware support

STM32G0 series with flash memmory at 64KB with UART debugging
STM32G0 series with flash memmory at 32KB without UART debugging

## Development tool

Arduino IDE

## Arduino Lib requirement 
none

## Installation
1. Configure the STM32 board support:
   - Open Arduino IDE
   - Go to File → Preferences
   - Add the following URL to "Additional Boards Manager URLs":
     ```
     https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
     ```
   - Click OK
2. Install STM32 board support:
   - Open Tools → Board → Boards Manager
   - Search for "STM32"
   - Install "STM32 MCU based boards"
   - Select an STM32G0 series board from Tools → Board → STM32 MCU based boards -> Generic STM32 G0 Series
