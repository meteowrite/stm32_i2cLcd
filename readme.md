# STM32 Liquid Crystal Display (LCD) via I2C driver 

Just a project with STM32F100 to interface an LCD display through PCF8574 over I2C. 
![Greetings](/Resources/readme_logo.jpg)

## Files
The entire project is implemented within [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html). 
Source and header files are located in **Src/i2c_lcd.c** and **Inc/i2c_lcd.h** respectively. 

### Latest updates
- Added configurability to cursor incrementation, blinking, display shifting. 
- Added support for custom generated characters in RAM (CGRAM). 
- Created _i2cLcd_ReadByte()_ task to read address counter (AC) and busy flag (BF). 

## Operation and dependencies
A custom data struct is used to allow multi-instance LCD addressing, each instance comprising of a handle to I2C HAL type, I2C Slave address and corresponding LCD parameters (display settings, cursors, backlight, etc).

Currently, the library functions for LCD control use the following HAL abstractions:
- HAL_I2C_Master_Transmit();
- HAL_I2C_Master_Receive();
- HAL_Delay();

## TODO:
- Refine delay tasks. Currently a delay of ~1ms is used between any commands, which is inefficient.
- Documentation!
- Include schematics and example code snippets
