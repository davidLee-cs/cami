/*
 * mxc4005.c
 *
 */

#include "mxc4005_registers.h"


void mxc4005_pd_write(uint16_t addr, uint16_t value){
	uint16_t mask;
	uint16_t readValue;
	uint16_t writeValue;
	uint16_t regmask;
	uint8_t size;
	/// <b>Algorithm of the method is as follows</b>
	mxc4005_I2C_write(addr,value); /// * Initiates I2C write to device
}

void mxc4005_deviceRegister_write(mxc4005_deviceRegister* devReg,int16_t value){
	uint16_t mask;
	uint16_t readValue;
	uint16_t writeValue;
	uint16_t regmask;
	uint8_t size;
	/// <b>Algorithm of the method is as follows</b>
	value=(uint16_t) value; /// * Converts the input value to uint16_t
	readValue=mxc4005_I2C_read(devReg->address);
	size=devReg->msb+1-devReg->lsb;
	mask=~(((uint16_t) -1)<<(size));
	writeValue=value&mask;
	value=value>>size;
	regmask=(~((((uint16_t) -1)<<(devReg->msb+1))^(((uint16_t) -1)<<devReg->lsb)))&0xFFFF; /// * Masks the bits for the new value to be written
	writeValue=(writeValue<<devReg->lsb)|(readValue&regmask);/// * Creates a new register value to be written to h/w based on created mask and data to be written
	if(readValue!=writeValue)
		mxc4005_I2C_write(devReg->address,writeValue); /// * Initiates I2C write to device
}
uint16_t mxc4005_deviceRegister_read(mxc4005_deviceRegister* devReg) {
	uint16_t readValue;
	uint16_t regmask;
	uint8_t size;
	uint8_t bitPos;
	uint16_t returnValue;

	returnValue=0;
	bitPos=0;
	/// <b>Algorithm of the method is as follows</b>
	readValue=mxc4005_I2C_read(devReg->address); ///	* Invoked an I2C Read
	size=devReg->msb+1-devReg->lsb;
	regmask=(((((uint16_t) -1)<<(devReg->msb+1))^(((uint16_t) -1)<<devReg->lsb)))&0xFFFF; /// * Masks the bits for the register value to be reported based on register positional information
	returnValue|=((readValue&regmask)>>devReg->lsb)<<bitPos; /// * Assembles the value of the register
	bitPos+=size;
	return returnValue; /// * Returns the value read from the h/w for the register name specified
}
void mxc4005_assignRegistermap(mxc4005_registers *devReg){
	devReg->CHORZ.address=0;
	devReg->CHORZ.msb=7;
	devReg->CHORZ.lsb=7;

	devReg->CHORXY.address=0;
	devReg->CHORXY.msb=6;
	devReg->CHORXY.lsb=6;

	devReg->SHYM.address=0;
	devReg->SHYM.msb=3;
	devReg->SHYM.lsb=3;

	devReg->SHYP.address=0;
	devReg->SHYP.msb=2;
	devReg->SHYP.lsb=2;

	devReg->SHXM.address=0;
	devReg->SHXM.msb=1;
	devReg->SHXM.lsb=1;

	devReg->SHXP.address=0;
	devReg->SHXP.msb=0;
	devReg->SHXP.lsb=0;

	devReg->ORZC.address=0;
	devReg->ORZC.msb=7;
	devReg->ORZC.lsb=7;

	devReg->ORXYC.address=0;
	devReg->ORXYC.msb=6;
	devReg->ORXYC.lsb=6;

	devReg->SHYMC.address=0;
	devReg->SHYMC.msb=3;
	devReg->SHYMC.lsb=3;

	devReg->SHYPC.address=0;
	devReg->SHYPC.msb=2;
	devReg->SHYPC.lsb=2;

	devReg->SHXMC.address=0;
	devReg->SHXMC.msb=1;
	devReg->SHXMC.lsb=1;

	devReg->SHXPC.address=0;
	devReg->SHXPC.msb=0;
	devReg->SHXPC.lsb=0;

	devReg->TILT.address=1;
	devReg->TILT.msb=7;
	devReg->TILT.lsb=7;

	devReg->ORZ.address=1;
	devReg->ORZ.msb=6;
	devReg->ORZ.lsb=6;

	devReg->ORXY.address=1;
	devReg->ORXY.msb=5;
	devReg->ORXY.lsb=4;

	devReg->DRDY.address=1;
	devReg->DRDY.msb=0;
	devReg->DRDY.lsb=0;

	devReg->SW_RST.address=1;
	devReg->SW_RST.msb=4;
	devReg->SW_RST.lsb=4;

	devReg->DRDYC.address=1;
	devReg->DRDYC.msb=0;
	devReg->DRDYC.lsb=0;

	devReg->ORD.address=2;
	devReg->ORD.msb=4;
	devReg->ORD.lsb=4;

	devReg->ORIZ.address=2;
	devReg->ORIZ.msb=3;
	devReg->ORIZ.lsb=2;

	devReg->ORIXY.address=2;
	devReg->ORIXY.msb=1;
	devReg->ORIXY.lsb=0;

	devReg->XOUT_UP.address=3;
	devReg->XOUT_UP.msb=7;
	devReg->XOUT_UP.lsb=0;

	devReg->XOUT_LOW.address=4;
	devReg->XOUT_LOW.msb=7;
	devReg->XOUT_LOW.lsb=4;

	devReg->YOUT_UP.address=5;
	devReg->YOUT_UP.msb=7;
	devReg->YOUT_UP.lsb=0;

	devReg->YOUT_LOW.address=6;
	devReg->YOUT_LOW.msb=7;
	devReg->YOUT_LOW.lsb=4;

	devReg->ZOUT_UP.address=7;
	devReg->ZOUT_UP.msb=7;
	devReg->ZOUT_UP.lsb=0;

	devReg->ZOUT_LOW.address=8;
	devReg->ZOUT_LOW.msb=7;
	devReg->ZOUT_LOW.lsb=4;

	devReg->TOUT.address=9;
	devReg->TOUT.msb=7;
	devReg->TOUT.lsb=0;

	devReg->ORZE.address=10;
	devReg->ORZE.msb=7;
	devReg->ORZE.lsb=7;

	devReg->ORXYE.address=10;
	devReg->ORXYE.msb=6;
	devReg->ORXYE.lsb=6;

	devReg->SHYME.address=10;
	devReg->SHYME.msb=3;
	devReg->SHYME.lsb=3;

	devReg->SHYPE.address=10;
	devReg->SHYPE.msb=2;
	devReg->SHYPE.lsb=2;

	devReg->SHXME.address=10;
	devReg->SHXME.msb=1;
	devReg->SHXME.lsb=1;

	devReg->SHXPE.address=10;
	devReg->SHXPE.msb=0;
	devReg->SHXPE.lsb=0;

	devReg->DRDYE.address=11;
	devReg->DRDYE.msb=0;
	devReg->DRDYE.lsb=0;

	devReg->SHM.address=12;
	devReg->SHM.msb=7;
	devReg->SHM.lsb=7;

	devReg->SHTH.address=12;
	devReg->SHTH.msb=6;
	devReg->SHTH.lsb=4;

	devReg->SHC.address=12;
	devReg->SHC.msb=3;
	devReg->SHC.lsb=2;

	devReg->ORC.address=12;
	devReg->ORC.msb=1;
	devReg->ORC.lsb=0;

	devReg->FSR.address=13;
	devReg->FSR.msb=6;
	devReg->FSR.lsb=5;

	devReg->Cksel.address=13;
	devReg->Cksel.msb=4;
	devReg->Cksel.lsb=4;

	devReg->PD.address=13;
	devReg->PD.msb=0;
	devReg->PD.lsb=0;

	devReg->WAI.address=15;
	devReg->WAI.msb=7;
	devReg->WAI.lsb=0;

}


