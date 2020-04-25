/*
 * i2c_lcd.h
 *
 *  Created on: 14 Aug 2019
 *      Author: dit
 */

#ifndef I2C_LCD_H_
#define I2C_LCD_H_

#define RS_SIGNAL_MASK			(1 << 0)
#define RW_SIGNAL_MASK			(1 << 1)
#define E_SIGNAL_MASK			(1 << 2)
#define BL_SIGNAL_MASK			(1 << 3)

#define I2CLCD_RS				(1 << 0)
#define I2CLCD_RW				(1 << 1)
#define I2CLCD_E				(1 << 2)
#define I2CLCD_BL				(1 << 3)


// Defines for command access
#define CLR_DISPLAY				(1<<0)
#define RET_HOME				(1<<1)
#define MODE_SET				(1<<2)
#define DISP_CTRL				(1<<3)
#define CD_SHIFT				(1<<4)
#define FUNC_SET				(1<<5)
#define GCRAM_ADDR				(1<<6)
#define DDRAM_ADDR				(1<<7)

#define MODE_SET_INCR			(1<<0)
#define MODE_SET_DECR			(0)
#define MODE_SET_DISP_SHIFT_ON	(1<<0)
#define MODE_SET_DISP_SHIFT_OFF	(0)

#define SHIFT					(1<<0)
#define DIR_INCR_DECR			(1<<1)

#define DISP_CTRL_BLINK_ON		(1<<0)
#define DISP_CTRL_BLINK_OFF		(0)

#define DISP_CTRL_CURSOR_ON		(1<<1)
#define DISP_CTRL_CURSOR_OFF	(0)

#define DISP_CTRL_DISPLAY_ON	(1<<2)
#define DISP_CTRL_DISPLAY_OFF	(0)

#define CD_SHIFT_DSIPLAY		(1<<3)
#define CD_SHIFT_CURSOR			(0)

#define CD_SHIFT_RIGHT			(1<<2)
#define CD_SHIFT_LEFT			(1<<2)

#define FUNC_SET_FO_5X10		(1<<2)
#define FUNC_SET_FO_5X8			0
#define FUNC_SET_LINES_2		(1<<3)
#define FUNC_SET_LINES_1		0
#define FUNC_SET_DLEN_8B		(1<<4)
#define FUNC_SET_DLEN_4B		0

// SendByte options for physical access
// opts[0]	0: Send Command (RS = 0)
//			1: Send Data (RS = 1)
// opts[1]	0: Send 8 bits (2 by 4bits)
// opts[1]	1: Send 4 bits
// opts[2]	0: Backlight On
// opts[2]	1: Backlight Off

#define I2CLCD_OPTS_RS		(1<<0)
#define I2CLCD_OPTS_DATA	I2CLCD_OPTS_RS
#define I2CLCD_OPTS_COMMAND	(0)

#define I2CLCD_OPTS_INIT	(1<<1)
#define I2CLCD_OPTS_NOINIT	(0)
#define I2CLCD_OPTS_4B		I2CLCD_OPTS_INIT
#define I2CLCD_OPTS_8B		I2CLCD_OPTS_NOINIT

#define I2CLCD_OPTS_BL_OFF	(1<<2)
#define I2CLCD_OPTS_BL_ON	(0)

#define I2CLCD_OPTS_WAIT_BF	(1<<3)

#define I2CLCD_MAX_BF_POLLS	127

#define DIR_INCR DIR_INCR_DECR & 0xFF

typedef struct i2cLcd_Stuct{
	I2C_HandleTypeDef *hi2c;
	uint8_t i2c_addr;
	uint8_t entry_mode_set;
	uint8_t diplay_ctrl;
	uint8_t cursor_shift;
	uint8_t function_set;
	uint8_t	cgram_addr;
	uint8_t ddram_addr;
	uint8_t blacklight;

}i2cLcd_HandleTypeDef;


uint8_t i2cLcd_CreateHandle(i2cLcd_HandleTypeDef *h_i2cLcd, I2C_HandleTypeDef *h_i2c, uint8_t i2c_slave_addr);
uint8_t i2cLcd_ClearDisplay(i2cLcd_HandleTypeDef * h_i2cLcd);
uint8_t i2cLcd_SendCmd(i2cLcd_HandleTypeDef * h_i2cLcd, uint8_t args);
uint8_t i2cLcd_RetHome(i2cLcd_HandleTypeDef * h_i2cLcd);
uint8_t i2cLcd_Init(i2cLcd_HandleTypeDef * h_i2cLcd);
uint8_t i2cLcd_SendByte(i2cLcd_HandleTypeDef * h_i2cLcd, uint8_t data, uint8_t opts);
uint8_t i2cLcd_WaitBusyFlag(i2cLcd_HandleTypeDef * h_i2cLcd);
uint8_t i2cLcd_SendChar(i2cLcd_HandleTypeDef * h_i2cLcd, uint8_t chr);
uint8_t i2cLcd_SetPos(i2cLcd_HandleTypeDef * h_i2cLcd, uint8_t pos);



#endif /* I2C_LCD_H_ */
