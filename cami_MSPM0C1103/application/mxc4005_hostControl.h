/*
 * mxc4005_hostControl.h
 *
 *  Created on: Sep 18, 2020
 *      Author: Karthik Rajagopal
 */

#ifndef MXC4005_HOSTCONTROL_H_
#define MXC4005_HOSTCONTROL_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"
#include "opt3007_hostControl.h"

#define MXC4005_BUSADDRESS 0x15

uint16_t mxc4005_I2C_read(uint8_t registerAddress);
void mxc4005_I2C_write(uint8_t registerAddress,uint16_t value);

#endif /* MXC4005_HOSTCONTROL_H_ */
