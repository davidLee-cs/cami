/*
 * mxc4005_functions.c
 *
 *  Created on: Sep 18, 2020
 *      Author: Karthik Rajagopal
 */

#include "mxc4005_functions.h"
#include "mxc4005_registers.h"

#define XOUT_UPPER      (0x03)
#define XOUT_LOWER      (0x04)
#define YOUT_UPPER      (0x05)
#define YOUT_LOWER      (0x06)
#define ZOUT_UPPER      (0x07)
#define ZOUT_LOWER      (0x08)

int16_t accel_x;
int16_t accel_y;
int16_t accel_z;

uint16_t axu, axl;
uint16_t ayu, ayl;
uint16_t azu, azl;

void mxc4005_readAccel(void){

	uint16_t readValue_u,readValue_l;

	readValue_u=mxc4005_I2C_read(XOUT_UPPER); ///
	axu = (readValue_u << 8) & 0xFF00;
	readValue_l = mxc4005_I2C_read(XOUT_LOWER); ///
	axl = readValue_l & 0x00FF;
	accel_x = (axu | axl) >> 4;
	if(accel_x > 2047)
	{
		accel_x = accel_x - 4096;
	}
 
 	// delay_cycles(10000);
    readValue_u=mxc4005_I2C_read(YOUT_UPPER); ///
	ayu = (readValue_u << 8) & 0xFF00;
	readValue_l=mxc4005_I2C_read(YOUT_LOWER); ///
	ayl |= readValue_l & 0x00FF;
	accel_y = (ayu | ayl) >> 4;
	if(accel_y > 2047)
	{
		accel_y = accel_y - 4096;
	}

 	// delay_cycles(10000);
    readValue_u=mxc4005_I2C_read(ZOUT_UPPER); ///
	azu = (readValue_u << 8) & 0xFF00;
	readValue_l=mxc4005_I2C_read(ZOUT_LOWER); ///	
	azl |= readValue_l & 0x00FF;
	accel_z = (azu | azl) >> 4;
	if(accel_z > 2047)
	{
		accel_z = accel_z - 4096;
	}

}

uint16_t mxc4005_readManufacturerID(mxc4005_registers* devReg){
	return mxc4005_deviceRegister_read(&devReg->WAI);
}

uint16_t mxc4005_powerDown(mxc4005_registers* devReg){
	 mxc4005_deviceRegister_write(&devReg->PD, 1);
}

uint16_t mxc4005_powerUp(mxc4005_registers* devReg){
	 mxc4005_deviceRegister_write(&devReg->PD, 0);
}

// uint16_t mxc4005_readDeviceID(mxc4005_registers* devReg){
// 	return mxc4005_deviceRegister_read(&devReg->ID);
// }

