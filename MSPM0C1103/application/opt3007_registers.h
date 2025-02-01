/*
 * opt3007.h
 *
 *  Created on: Sep 17, 2020
 *      Author: Karthik Rajagopal
 */

#ifndef OPT3007_REGISTERS_H_
#define OPT3007_REGISTERS_H_


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "opt3007_hostControl.h"

struct ti_opt3007_deviceRegister{
	uint8_t msb;
	uint8_t lsb;
	uint8_t address;
};

typedef struct ti_opt3007_deviceRegister ti_opt3007_deviceRegister;

struct ti_opt3007_registers{
	ti_opt3007_deviceRegister R;
	ti_opt3007_deviceRegister E;
	ti_opt3007_deviceRegister RN;
	ti_opt3007_deviceRegister CT;
	ti_opt3007_deviceRegister M;
	ti_opt3007_deviceRegister OVF;
	ti_opt3007_deviceRegister CRF;
	ti_opt3007_deviceRegister FH;
	ti_opt3007_deviceRegister FL;
	ti_opt3007_deviceRegister L;
	ti_opt3007_deviceRegister POL;
	ti_opt3007_deviceRegister ME;
	ti_opt3007_deviceRegister FC;
	ti_opt3007_deviceRegister LE;
	ti_opt3007_deviceRegister TL;
	ti_opt3007_deviceRegister HE;
	ti_opt3007_deviceRegister TH;
	ti_opt3007_deviceRegister ID;
	ti_opt3007_deviceRegister DID;
};

typedef struct ti_opt3007_registers      ti_opt3007_registers;

void     ti_opt3007_deviceRegister_write(ti_opt3007_deviceRegister *devReg,int16_t value);
uint16_t ti_opt3007_deviceRegister_read(ti_opt3007_deviceRegister *devReg);
void     ti_opt3007_assignRegistermap(ti_opt3007_registers* devReg);

#endif /* OPT3007_REGISTERS_H_ */
