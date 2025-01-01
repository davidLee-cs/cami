/*
 * mxc4005_functions.c
 *
 *  Created on: Sep 18, 2020
 *      Author: Karthik Rajagopal
 */

#include "mxc4005_functions.h"

#define XOUT_UPPER      (0x03)
#define XOUT_LOWER      (0x04)
#define YOUT_UPPER      (0x05)
#define YOUT_LOWER      (0x06)
#define ZOUT_UPPER      (0x07)
#define ZOUT_LOWER      (0x08)

int16_t accel_x;
int16_t accel_y;
int16_t accel_z;


void mxc4005_readLux(void){

	uint16_t readValue,mantissa;

	readValue=mxc4005_I2C_read(XOUT_UPPER); ///
	accel_x = (readValue << 8) & 0xFF00;
	readValue=mxc4005_I2C_read(XOUT_LOWER); ///
	accel_x |= readValue & 0x00FF;
 	
    readValue=mxc4005_I2C_read(YOUT_UPPER); ///
	accel_y = (readValue << 8) & 0xFF00;
	readValue=mxc4005_I2C_read(YOUT_LOWER); ///
	accel_y |= readValue & 0x00FF;
 	
    readValue=mxc4005_I2C_read(ZOUT_UPPER); ///
	accel_z = (readValue << 8) & 0xFF00;
	readValue=mxc4005_I2C_read(ZOUT_LOWER); ///	
	accel_z |= readValue & 0x00FF;

}

uint16_t mxc4005_readManufacturerID(mxc4005_registers* devReg){
	return mxc4005_deviceRegister_read(&devReg->WAI);
}


