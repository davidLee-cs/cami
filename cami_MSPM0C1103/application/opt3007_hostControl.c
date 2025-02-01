/*
 * opt3007_hostControl.c
 *
 *  Created on: Sep 18, 2020
 *      Author: Karthik Rajagopal
 */

#include "opt3007_hostControl.h"


uint16_t ti_opt3007_I2C_read(uint8_t registerAddress){
	//I2C read operation and return uint16_t value as per datasheet
	// Data should be in first byte received from I2C denotes 15:8 and 2nd byte from I2C denotes 7:0
	uint16_t returnValue=0;
	uint16_t gRxLen = 2;	// default

	ti_opt3007_I2C_write(registerAddress, 0);	// for read Register

    /* Send a read request to Target */
    DL_I2C_startControllerTransfer(I2C_INST, TI_OPT3007_BUSADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_RX, gRxLen);

    for (uint8_t i = 0; i < gRxLen; i++) {
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_INST))
            ;
        gRxPacket[i] = DL_I2C_receiveControllerData(I2C_INST);
    }

	returnValue = gRxPacket[0] << 8 | gRxPacket[1];
	return returnValue;

}
void ti_opt3007_I2C_write(uint8_t registerAddress,uint16_t value){
	//I2C write operation to the denoted register
	// Data should be in first byte sent on I2C denotes 15:8 and 2nd byte sent to I2C denotes 7:0
	uint32_t gTxLen, gTxCount;
	uint16_t length;

	if(value == 0)
	{
		length = 1;		// for read mode
        //  __BKPT(0);
	}
	else
	{
		length = 3;
	}

	uint8_t txBuffer[1 + length];
    // uint8_t txBuffer[length];
	
	txBuffer[0] = registerAddress;	// First byte is register address
	txBuffer[1] = (value >> 8) & 0x0FF;
	txBuffer[2] = value & 0xFF;
        // __BKPT(0);

	// gTxLen = 1 + length;
	gTxLen = 0 + length;

    /*
     * Fill FIFO with data. This example will send a MAX of 4 bytes since it
     * doesn't handle the case where FIFO is full
     */
    DL_I2C_fillControllerTXFIFO(I2C_INST, txBuffer, gTxLen);

    /* Wait for I2C to be Idle */
    while (!(
        DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE))
        ;

    /* Send the packet to the controller.
     * This function will send Start + Stop automatically.
     */
    DL_I2C_startControllerTransfer(I2C_INST, TI_OPT3007_BUSADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX, gTxLen);

    /* Poll until the Controller writes all bytes */
    while (DL_I2C_getControllerStatus(I2C_INST) &
           DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
        ;

    /* Trap if there was an error */
    if (DL_I2C_getControllerStatus(I2C_INST) &
        DL_I2C_CONTROLLER_STATUS_ERROR) {
        /* LED will remain high if there is an error */
        __BKPT(0);
    }

    /* Wait for I2C to be Idle */
    while (!(
        DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE))
        ;
	// delay_cycles(1000);

}
