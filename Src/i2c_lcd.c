/*
 * i2c_lcd.c
 *
 *  Created on: 14 Aug 2019
 *      Author: dit
 */

#include "main.h"
#include "i2c_lcd.h"


uint8_t i2cLcd_SendByte(i2cLcd_HandleTypeDef * h_i2cLcd, uint8_t data, uint8_t opts){
	// Opts[0] - R/S select
	// Opts[1] -initialization

	HAL_StatusTypeDef hal_stat;
	uint8_t i2c_frame_data[5];
	uint8_t cmd;
	uint8_t wait_bf;
	uint8_t i2c_frame_size;
	uint8_t n;

	// Select between command/data frame
	cmd = opts & I2CLCD_OPTS_RS;

	// Toggle between waiting 1ms or poll Busy Flag
	wait_bf = opts & I2CLCD_OPTS_WAIT_BF;


	// Frame size for I2C communication.
	if (opts & I2CLCD_OPTS_4B) {
		i2c_frame_size = 2;
	}
	else {
		i2c_frame_size = 4;
	}

	// Add final all 1s on the 4 data bits to be able to read BF after a transaction
	i2c_frame_size += wait_bf;


	i2c_frame_data[0] = (data & 0xF0) | (I2CLCD_RS & cmd) | I2CLCD_BL | (I2CLCD_E);
	i2c_frame_data[1] = i2c_frame_data[0] & (~I2CLCD_E);

	i2c_frame_data[2] = ((data << 4) & 0xF0) | (I2CLCD_RS & cmd) | I2CLCD_BL | (I2CLCD_E);
	i2c_frame_data[3] = i2c_frame_data[2] & (~I2CLCD_E);

	if(wait_bf)
		i2c_frame_data[i2c_frame_size-1] = i2c_frame_data[i2c_frame_size-2] | 0x80;

	// HAL transmits i2c_frame_data[0],[1], ... , i2c_frame_data[i2c_frame_size-1]
	hal_stat = HAL_I2C_Master_Transmit(h_i2cLcd->hi2c, h_i2cLcd->i2c_addr, i2c_frame_data,
										i2c_frame_size, 10);

	//i2cLcd_WaitBusyFlag();
	if (wait_bf) {
		n = 0;
		do {
			hal_stat |= HAL_I2C_Master_Receive(h_i2cLcd->hi2c, h_i2cLcd->i2c_addr, i2c_frame_data,
										1, 10);
			n++;
		} while ( (n < I2CLCD_MAX_BF_POLLS) && (i2c_frame_data[0] & 0x80) );
	}
	else {
		HAL_Delay(1);
	}

	return (uint8_t) hal_stat;

}



//uint8_t i2cLcd_WaitBusyFlag(){
//	HAL_StatusTypeDef hal_stat;
//
//	uint8_t i;
//	uint8_t r_bf[4];
//	uint8_t rd[2];
//	uint8_t bf;
//	uint8_t ac;
//
//	i = 0;
//	do {
//		//r_bf[0] =  RW_SIGNAL_MASK| E_SIGNAL_MASK | BL_SIGNAL_MASK;
//		//r_bf[1] =  r_bf[0] & (~E_SIGNAL_MASK); // generate negedge on E
//
//		r_bf[0] = (0xF0) | ( BL_SIGNAL_MASK | E_SIGNAL_MASK | RW_SIGNAL_MASK); // no RS`
//		r_bf[1] = r_bf[0] & (~E_SIGNAL_MASK); // generate negedge on E
//		r_bf[2] = r_bf[0];
//		r_bf[3] = r_bf[0] & (~RW_SIGNAL_MASK);
//
//		hal_stat  = HAL_I2C_Master_Transmit(_hi2c, _i2c_addr, &r_bf[1], 2, 10);
//		hal_stat |= HAL_I2C_Master_Receive(_hi2c, _i2c_addr, &rd[0], 1, 10);
//
//		hal_stat |= HAL_I2C_Master_Transmit(_hi2c, _i2c_addr, &r_bf[1], 2, 10);
//		hal_stat |= HAL_I2C_Master_Receive(_hi2c, _i2c_addr, &rd[1], 1, 10);
//
//		//hal_stat |= HAL_I2C_Master_Transmit(_hi2c, _i2c_addr, &r_bf[3], 1, 10);
//
//		bf = 0x80 & rd[0];
//		ac = (0x70 & rd[0]) | ((0xF0 & rd[1])>>4) ;
//		i++;
//	} while (bf);
//
//	return hal_stat;
//
//}




//uint8_t i2cLcd_SendCmd(uint8_t args){
//	return i2cLcd_SendByte(args, 0, 0);
//}
//
//uint8_t i2cLcd_ClearDisplay(){
//	return i2cLcd_SendCmd(CLR_DISPLAY);
//}
//
//uint8_t i2cLcd_RetHome(){
//	return i2cLcd_SendCmd(RET_HOME);
//}
//
//
//uint8_t i2cLcd_Init(i2cLcd_TypeDef *i2c, I2C_HandleTypeDef *){
//
//	_i2c_addr = i2c_addr;
//
//	// Very first initialization
//	i2cLcd_SendByte(FUNC_SET, 0, 1);
//
//}




uint8_t i2cLcd_Init(i2cLcd_HandleTypeDef * h_i2cLcd){
	uint8_t ret;
	h_i2cLcd->function_set = FUNC_SET | FUNC_SET_DLEN_8B;

	ret = 0;
	ret |= i2cLcd_SendByte(h_i2cLcd, h_i2cLcd->function_set, I2CLCD_OPTS_INIT);
	HAL_Delay(5);
	ret |= i2cLcd_SendByte(h_i2cLcd, h_i2cLcd->function_set, I2CLCD_OPTS_INIT);
	HAL_Delay(1);
	ret |= i2cLcd_SendByte(h_i2cLcd, h_i2cLcd->function_set, I2CLCD_OPTS_INIT);
	HAL_Delay(1);


	h_i2cLcd->function_set = FUNC_SET | FUNC_SET_DLEN_4B;
	ret |= i2cLcd_SendByte(h_i2cLcd, h_i2cLcd->function_set, I2CLCD_OPTS_INIT);
	HAL_Delay(1);

	h_i2cLcd->function_set = FUNC_SET | FUNC_SET_DLEN_4B | FUNC_SET_LINES_2 | FUNC_SET_FO_5X8;
	ret |= i2cLcd_SendByte(h_i2cLcd, h_i2cLcd->function_set, I2CLCD_OPTS_NOINIT);
	HAL_Delay(1);
	i2cLcd_ClearDisplay(h_i2cLcd);
	HAL_Delay(5);
	return ret;
}

uint8_t i2cLcd_ClearDisplay(i2cLcd_HandleTypeDef * h_i2cLcd){

	return i2cLcd_SendByte(h_i2cLcd, CLR_DISPLAY, 0);
}

uint8_t i2cLcd_SendChar(i2cLcd_HandleTypeDef * h_i2cLcd, uint8_t chr){

	return i2cLcd_SendByte(h_i2cLcd, chr, I2CLCD_OPTS_DATA);
}

uint8_t i2cLcd_SetPos(i2cLcd_HandleTypeDef * h_i2cLcd, uint8_t pos){
	return i2cLcd_SendByte(h_i2cLcd, DDRAM_ADDR | pos, I2CLCD_OPTS_NOINIT);
}

uint8_t i2cLcd_CreateHandle(i2cLcd_HandleTypeDef *h_i2cLcd, I2C_HandleTypeDef *h_i2c, uint8_t i2c_slave_addr){

	uint8_t init_state;
	init_state = 0x00; // all inputs of PCF
	// Bind I2C HAL handler
	if (h_i2c == NULL)
		return -1;

	h_i2cLcd->hi2c = h_i2c;

	// Set slave address
	h_i2cLcd->i2c_addr = i2c_slave_addr;

	return HAL_I2C_Master_Transmit(h_i2cLcd->hi2c, h_i2cLcd->i2c_addr, &init_state, 1, 10);
}



