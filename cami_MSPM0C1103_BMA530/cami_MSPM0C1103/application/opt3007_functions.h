/*
 * opt3007_functions.h
 *
 *  Created on: Sep 18, 2020
 *      Author: Karthik Rajagopal
 */

#ifndef OPT3007_FUNCTIONS_H_
#define OPT3007_FUNCTIONS_H_

#include "opt3007_registers.h"


void ti_opt3007_setSensorShutDown(ti_opt3007_registers* devReg);
void ti_opt3007_setSensorSingleShot(ti_opt3007_registers* devReg);
void ti_opt3007_setSensorContinuous(ti_opt3007_registers* devReg);
void ti_opt3007_setSensorConversionTime100mS(ti_opt3007_registers* devReg);
void ti_opt3007_setSensorConversionTime800mS(ti_opt3007_registers* devReg);
double ti_opt3007_readLux(void);
uint16_t ti_opt3007_readManufacturerID(ti_opt3007_registers* devReg);
uint16_t ti_opt3007_readDeviceID(ti_opt3007_registers* devReg);
void ti_opt3007_setRn(ti_opt3007_registers* devReg);


#endif /* OPT3007_FUNCTIONS_H_ */
