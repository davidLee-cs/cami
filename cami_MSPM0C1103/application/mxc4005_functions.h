/*
 * mxc4005_functions.h
 *
 *  Created on: Sep 18, 2020
 *      Author: Karthik Rajagopal
 */

#ifndef MXC4005_FUNCTIONS_H_
#define MXC4005_FUNCTIONS_H_

#include "mxc4005_registers.h"

void mxc4005_readAccel(void);
uint16_t mxc4005_readManufacturerID(mxc4005_registers* devReg);
uint16_t mxc4005_readDeviceID(mxc4005_registers* devReg);

extern int16_t accel_x;
extern int16_t accel_y;
extern int16_t accel_z;


#endif /* MXC4005_FUNCTIONS_H_ */
