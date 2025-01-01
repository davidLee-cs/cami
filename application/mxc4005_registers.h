/*
 * mxc4005.h
 *
 *  Created on: Sep 17, 2020
 *      Author: Karthik Rajagopal
 */

#ifndef MXC4005_REGISTERS_H_
#define MXC4005_REGISTERS_H_


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "mxc4005_hostControl.h"

struct mxc4005_deviceRegister{
	uint8_t msb;
	uint8_t lsb;
	uint8_t address;
};

typedef struct mxc4005_deviceRegister mxc4005_deviceRegister;

struct mxc4005_registers{
	mxc4005_deviceRegister CHORZ;
	mxc4005_deviceRegister CHORXY;
	mxc4005_deviceRegister SHYM;
	mxc4005_deviceRegister SHYP;
	mxc4005_deviceRegister SHXM;
	mxc4005_deviceRegister SHXP;
	mxc4005_deviceRegister ORZC;
	mxc4005_deviceRegister ORXYC;
	mxc4005_deviceRegister SHYMC;
	mxc4005_deviceRegister SHYPC;
	mxc4005_deviceRegister SHXMC;
	mxc4005_deviceRegister SHXPC;
	mxc4005_deviceRegister TILT;
	mxc4005_deviceRegister ORZ;
	mxc4005_deviceRegister ORXY;
	mxc4005_deviceRegister DRDY;
	mxc4005_deviceRegister SW_RST;
	mxc4005_deviceRegister DRDYC;
	
    mxc4005_deviceRegister ORD;
	mxc4005_deviceRegister ORIZ;
	mxc4005_deviceRegister ORIXY;
	mxc4005_deviceRegister XOUT_UP;
	mxc4005_deviceRegister XOUT_LOW;
    mxc4005_deviceRegister YOUT_UP;
	mxc4005_deviceRegister YOUT_LOW;
    mxc4005_deviceRegister ZOUT_UP;
	mxc4005_deviceRegister ZOUT_LOW;
    mxc4005_deviceRegister TOUT;
	mxc4005_deviceRegister ORZE;
	mxc4005_deviceRegister ORXYE;
	mxc4005_deviceRegister SHYME;
	mxc4005_deviceRegister SHYPE;
	mxc4005_deviceRegister SHXME;
	mxc4005_deviceRegister SHXPE;
	mxc4005_deviceRegister DRDYE;
	mxc4005_deviceRegister SHM;
	mxc4005_deviceRegister SHTH;
	mxc4005_deviceRegister SHC;
	mxc4005_deviceRegister ORC;
	mxc4005_deviceRegister FSR;
	mxc4005_deviceRegister Cksel;
	mxc4005_deviceRegister PD;
	mxc4005_deviceRegister WAI;        
};

typedef struct mxc4005_registers      mxc4005_registers;

void     mxc4005_deviceRegister_write(mxc4005_deviceRegister *devReg,int16_t value);
uint16_t mxc4005_deviceRegister_read(mxc4005_deviceRegister *devReg);
void     mxc4005_assignRegistermap(mxc4005_registers* devReg);

#endif /* MXC4005_REGISTERS_H_ */
