/*
 * bma530.h
 *
 *  Created on: Sep 17, 2020
 *      Author: Karthik Rajagopal
 */

#ifndef bma530_REGISTERS_H_
#define bma530_REGISTERS_H_


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "bma530_hostControl.h"

struct bma530_deviceRegister{
	uint8_t msb;
	uint8_t lsb;
	uint8_t address;
};

typedef struct bma530_deviceRegister bma530_deviceRegister;

struct bma530_registers{
	bma530_deviceRegister acc_data_rdy;
	bma530_deviceRegister temperature_rdy;
	bma530_deviceRegister sensor_rdy;
	bma530_deviceRegister SHYP;
	bma530_deviceRegister SHXM;
	bma530_deviceRegister SHXP;
	bma530_deviceRegister ORZC;
	bma530_deviceRegister ORXYC;
	bma530_deviceRegister SHYMC;
	bma530_deviceRegister SHYPC;
	bma530_deviceRegister SHXMC;
	bma530_deviceRegister SHXPC;
	bma530_deviceRegister TILT;
	bma530_deviceRegister ORZ;
	bma530_deviceRegister ORXY;
	bma530_deviceRegister DRDY;
	bma530_deviceRegister SW_RST;
	bma530_deviceRegister DRDYC;	
    bma530_deviceRegister ORD;
	bma530_deviceRegister ORIZ;
	bma530_deviceRegister ORIXY;
	bma530_deviceRegister XOUT_UP;
	bma530_deviceRegister XOUT_LOW;
    bma530_deviceRegister YOUT_UP;
	bma530_deviceRegister YOUT_LOW;
    bma530_deviceRegister ZOUT_UP;
	bma530_deviceRegister ZOUT_LOW;
    bma530_deviceRegister TOUT;
	bma530_deviceRegister ORZE;
	bma530_deviceRegister ORXYE;
	bma530_deviceRegister SHYME;
	bma530_deviceRegister SHYPE;
	bma530_deviceRegister SHXME;
	bma530_deviceRegister SHXPE;
	bma530_deviceRegister DRDYE;
	bma530_deviceRegister SHM;
	bma530_deviceRegister SHTH;
	bma530_deviceRegister SHC;
	bma530_deviceRegister ORC;
	bma530_deviceRegister FSR;
	bma530_deviceRegister Cksel;
	bma530_deviceRegister PD;
	bma530_deviceRegister WAI;        
};

typedef struct bma530_registers      bma530_registers;

void bma530_byte_write(uint16_t addr, uint16_t value);
// void bma530_pd_write(bma530_deviceRegister* devReg, uint16_t value);
void     bma530_deviceRegister_write(bma530_deviceRegister *devReg,int16_t value);
uint16_t bma530_deviceRegister_read(bma530_deviceRegister *devReg);
void bma530_assignRegistermap(bma530_registers *devReg);

#endif /* bma530_REGISTERS_H_ */
