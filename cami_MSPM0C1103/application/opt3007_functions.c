/*
 * opt3007_functions.c
 *
 *  Created on: Sep 18, 2020
 *      Author: Karthik Rajagopal
 */

#include "opt3007_functions.h"

void ti_opt3007_setSensorShutDown(ti_opt3007_registers* devReg){
	ti_opt3007_deviceRegister_write(&devReg->M,0);
}
void ti_opt3007_setSensorSingleShot(ti_opt3007_registers* devReg){
	ti_opt3007_deviceRegister_write(&devReg->M,1);
}
void ti_opt3007_setSensorContinuous(ti_opt3007_registers* devReg){
	ti_opt3007_deviceRegister_write(&devReg->M,3);
}
void ti_opt3007_setSensorConversionTime100mS(ti_opt3007_registers* devReg){
	ti_opt3007_deviceRegister_write(&devReg->CT,0);
}
void ti_opt3007_setSensorConversionTime800mS(ti_opt3007_registers* devReg){
	ti_opt3007_deviceRegister_write(&devReg->CT,1);
}

void ti_opt3007_setRn(ti_opt3007_registers* devReg){
	ti_opt3007_deviceRegister_write(&devReg->RN,12);  // 0 -> 12 변경  2025.06.03
}


uint16_t readValue,mantissa;
uint8_t myexp;
double lux;

double ti_opt3007_readLux(void){



	readValue=ti_opt3007_I2C_read(0x00); ///	* Invoked an I2C Read
	mantissa=readValue&0x0FFF;
	myexp=readValue>>12;
	lux=10e-3*(((uint16_t) 1)<<myexp)*mantissa;
	return lux;
}

uint16_t ti_opt3007_readManufacturerID(ti_opt3007_registers* devReg){
	return ti_opt3007_deviceRegister_read(&devReg->ID);
}
uint16_t ti_opt3007_readDeviceID(ti_opt3007_registers* devReg){
	return ti_opt3007_deviceRegister_read(&devReg->DID);
}

