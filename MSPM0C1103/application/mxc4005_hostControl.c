/*
 * mxc4005_hostControl.c
 *
 */

#include "mxc4005_hostControl.h"

uint16_t mxc4005_I2C_read(uint8_t registerAddress){
	//I2C read operation and return uint16_t value as per datasheet
	// Data should be in first byte received from I2C denotes 15:8 and 2nd byte from I2C denotes 7:0
	uint16_t returnValue=0;
	gRxLen = 2;	// default

	mxc4005_I2C_write(registerAddress, 0);	// for read Register
	
	/* Step 2: Initiate read by sending START again and switch to RX mode */
	gI2cControllerStatus = I2C_STATUS_RX_STARTED;
	DL_I2C_startControllerTransfer(I2C_INST, MXC4005_BUSADDRESS, DL_I2C_CONTROLLER_DIRECTION_RX, gRxLen);   // gRxLen :2

	/* Wait for data to be received */
	while (gI2cControllerStatus != I2C_STATUS_RX_COMPLETE) {
		__WFE();
	}

	while (DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);

	/* Handle errors */
	if (DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
		__BKPT(0); // Error breakpoint
	}

	returnValue = gRxPacket[0] << 8 | gRxPacket[1];
	return returnValue;

}
void mxc4005_I2C_write(uint8_t registerAddress,uint16_t value){
	
    // uint32_t gTxLen, gTxCount;
	uint16_t length;

	if(value == 0)
	{
		length = 1;		// for read mode
	}
	else
	{
		length = 3;
	}

	uint8_t txBuffer[1 + length];
	
	txBuffer[0] = registerAddress;	// First byte is register address
	txBuffer[1] = (value >> 8) & 0x0FF;
	txBuffer[2] = value & 0xFF;

	
//	for (uint32_t i = 0; i < length; i++) {
//		txBuffer[i + 1] = data[i];	// Following bytes are data
//	}

	//gTxLen = 1 + length;
	gTxLen = 0 + length;

	gTxCount = DL_I2C_fillControllerTXFIFO(I2C_INST, txBuffer, gTxLen);
	
	/* Enable TXFIFO trigger interrupt if there are more bytes to send */
	if (gTxCount < gTxLen) {
		DL_I2C_enableInterrupt(I2C_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
	} else {
		DL_I2C_disableInterrupt(I2C_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
	}

	/* Start the transfer (with START + STOP condition) */
	gI2cControllerStatus = I2C_STATUS_TX_STARTED;
	while (!(DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
	DL_I2C_startControllerTransfer(I2C_INST, MXC4005_BUSADDRESS, DL_I2C_CONTROLLER_DIRECTION_TX, gTxLen);

	/* Wait for completion */
	while ((gI2cControllerStatus != I2C_STATUS_TX_COMPLETE) && (gI2cControllerStatus != I2C_STATUS_ERROR)) {
		__WFE();
	}

	while (DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);

	/* Handle errors */
	if (DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
		__BKPT(0); // Error breakpoint
	}

	while (!(DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
	delay_cycles(1000);

}
