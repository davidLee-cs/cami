/*
 * opt3007.c
 *
 *  Created on: Sep 17, 2020
 *      Author: Karthik Rajagopal
 */

#include "opt3007_registers.h"

void ti_opt3007_deviceRegister_write(ti_opt3007_deviceRegister* devReg,int16_t value){
	uint16_t mask;
	uint16_t readValue;
	uint16_t writeValue;
	uint16_t regmask;
	uint8_t size;
	/// <b>Algorithm of the method is as follows</b>
	value=(uint16_t) value; /// * Converts the input value to uint16_t
	readValue=ti_opt3007_I2C_read(devReg->address);
	size=devReg->msb+1-devReg->lsb;
	mask=~(((uint16_t) -1)<<(size));
	writeValue=value&mask;
	value=value>>size;
	regmask=(~((((uint16_t) -1)<<(devReg->msb+1))^(((uint16_t) -1)<<devReg->lsb)))&0xFFFF; /// * Masks the bits for the new value to be written
	writeValue=(writeValue<<devReg->lsb)|(readValue&regmask);/// * Creates a new register value to be written to h/w based on created mask and data to be written
	if(readValue!=writeValue)
		ti_opt3007_I2C_write(devReg->address,writeValue); /// * Initiates I2C write to device
}
uint16_t ti_opt3007_deviceRegister_read(ti_opt3007_deviceRegister* devReg) {
	uint16_t readValue;
	uint16_t regmask;
	uint8_t size;
	uint8_t bitPos;
	uint16_t returnValue;

	returnValue=0;
	bitPos=0;
	/// <b>Algorithm of the method is as follows</b>
	readValue=ti_opt3007_I2C_read(devReg->address); ///	* Invoked an I2C Read
	size=devReg->msb+1-devReg->lsb;
	regmask=(((((uint16_t) -1)<<(devReg->msb+1))^(((uint16_t) -1)<<devReg->lsb)))&0xFFFF; /// * Masks the bits for the register value to be reported based on register positional information
	returnValue|=((readValue&regmask)>>devReg->lsb)<<bitPos; /// * Assembles the value of the register
	bitPos+=size;
	return returnValue; /// * Returns the value read from the h/w for the register name specified
}
void ti_opt3007_assignRegistermap(ti_opt3007_registers* devReg){
	devReg->E.address=0;
	devReg->E.msb=15;
	devReg->E.lsb=12;

	devReg->R.address=0;
	devReg->R.msb=11;
	devReg->R.lsb=0;

	devReg->RN.address=1;
	devReg->RN.msb=15;
	devReg->RN.lsb=12;

	devReg->CT.address=1;
	devReg->CT.msb=11;
	devReg->CT.lsb=11;

	devReg->M.address=1;
	devReg->M.msb=10;
	devReg->M.lsb=9;

	devReg->OVF.address=1;
	devReg->OVF.msb=8;
	devReg->OVF.lsb=8;

	devReg->CRF.address=1;
	devReg->CRF.msb=7;
	devReg->CRF.lsb=7;

	devReg->FH.address=1;
	devReg->FH.msb=6;
	devReg->FH.lsb=6;

	devReg->FL.address=1;
	devReg->FL.msb=5;
	devReg->FL.lsb=5;

	devReg->L.address=1;
	devReg->L.msb=4;
	devReg->L.lsb=4;

	devReg->POL.address=1;
	devReg->POL.msb=3;
	devReg->POL.lsb=3;

	devReg->ME.address=1;
	devReg->ME.msb=2;
	devReg->ME.lsb=2;

	devReg->FC.address=1;
	devReg->FC.msb=1;
	devReg->FC.lsb=0;

	devReg->LE.address=2;
	devReg->LE.msb=15;
	devReg->LE.lsb=12;

	devReg->TL.address=2;
	devReg->TL.msb=11;
	devReg->TL.lsb=0;

	devReg->HE.address=3;
	devReg->HE.msb=15;
	devReg->HE.lsb=12;

	devReg->TH.address=3;
	devReg->TH.msb=11;
	devReg->TH.lsb=0;

	devReg->ID.address=0x7E;
	devReg->ID.msb=15;
	devReg->ID.lsb=0;

	devReg->DID.address=0x7F;
	devReg->DID.msb=15;
	devReg->DID.lsb=0;
}


