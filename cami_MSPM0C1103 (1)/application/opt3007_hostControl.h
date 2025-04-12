/*
 * opt3007_hostControl.h
 *
 *  Created on: Sep 18, 2020
 *      Author: Karthik Rajagopal
 */

#ifndef OPT3007_HOSTCONTROL_H_
#define OPT3007_HOSTCONTROL_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

#define TI_OPT3007_BUSADDRESS 0x45

/* Maximum size of TX packet */
#define I2C_TX_MAX_PACKET_SIZE (16)

/* Number of bytes to send to target device */
#define I2C_TX_PACKET_SIZE (16)

/* Maximum size of RX packet */
#define I2C_RX_MAX_PACKET_SIZE (16)

/* Number of bytes to received from target */
#define I2C_RX_PACKET_SIZE (16)

/* I2C Target address */
#define I2C_TARGET_ADDRESS (0x48)

/* Data sent to the Target */
extern uint8_t gTxPacket[I2C_TX_MAX_PACKET_SIZE];
/* Counters for TX length and bytes sent */
extern uint32_t gTxLen, gTxCount;

/* Data received from Target */
extern uint8_t gRxPacket[I2C_RX_MAX_PACKET_SIZE];
/* Counters for TX length and bytes sent */
extern uint32_t gRxLen, gRxCount;

/* Indicates status of I2C */
enum I2cControllerStatus {
    I2C_STATUS_IDLE = 0,
    I2C_STATUS_TX_STARTED,
    I2C_STATUS_TX_INPROGRESS,
    I2C_STATUS_TX_COMPLETE,
    I2C_STATUS_RX_STARTED,
    I2C_STATUS_RX_INPROGRESS,
    I2C_STATUS_RX_COMPLETE,
    I2C_STATUS_ERROR,
} gI2cControllerStatus;


uint16_t ti_opt3007_I2C_read(uint8_t registerAddress);
void ti_opt3007_I2C_write(uint8_t registerAddress,uint16_t value);

#endif /* OPT3007_HOSTCONTROL_H_ */
