/**
 * Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <stdint.h>
#include <stdint.h>
#include <stdlib.h>
// #include <stdio.h>

#include "bma5.h"
// #include "coines.h"
#include "common.h"
#include "ti_msp_dl_config.h"


/* Variable to store the device address */
static uint8_t dev_addr;

int8_t mspm0c1103_I2C_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t mspm0c1103_I2C_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void delay_usec(uint32_t delay);

/*!
 * @brief I2C read function map to COINES platform
 */
BMA5_INTF_RET_TYPE bma5_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    return mspm0c1103_I2C_read(dev_addr, reg_addr, reg_data, (uint16_t)len);
}

/*!
 * @brief I2C write function map to COINES platform
 */
BMA5_INTF_RET_TYPE bma5_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    return mspm0c1103_I2C_write(dev_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
}

/*!
 * @brief SPI read function map to COINES platform
 */
BMA5_INTF_RET_TYPE bma5_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    // return coines_read_spi(COINES_SPI_BUS_0, dev_addr, reg_addr, reg_data, (uint16_t)len);
}

/*!
 * @brief SPI write function map to COINES platform
 */
BMA5_INTF_RET_TYPE bma5_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    // return coines_write_spi(COINES_SPI_BUS_0, dev_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
}

/*!
 * @brief Delay function map to COINES platform
 */
void bma5_delay_us(uint32_t period, void *intf_ptr)
{   
    uint32_t i;
    for(i=0; i<period; i++)
    {
        delay_cycles(24);   // 1usec    // 24MHz 
    }
    
}

void bma5_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMA5_OK:

            /* Do nothing */
            break;
        case BMA5_E_NULL_PTR:
            // printf("API name %s\t", api_name);
            // printf("Error  [%d] : Null pointer\r\n", rslt);
            break;
        case BMA5_E_COM_FAIL:
            // printf("API name %s\t", api_name);
            // printf("Error  [%d] : Communication failure\r\n", rslt);
            break;
        case BMA5_E_DEV_NOT_FOUND:
            // printf("API name %s\t", api_name);
            // printf("Error  [%d] : Device not found\r\n", rslt);
            break;
        default:
            // printf("API name %s\t", api_name);
            // printf("Error  [%d] : Unknown error code\r\n", rslt);
            break;
    }
}

int8_t bma5_interface_init(struct bma5_dev *bma5, uint8_t intf, enum bma5_context context)
{
    int8_t rslt = BMA5_OK;

    if (bma5 != NULL)
    {
        // int16_t result = coines_open_comm_intf(COINES_COMM_INTF_USB, NULL);

        // if (result < COINES_SUCCESS)
        // {
        //     printf(
        //         "\n Unable to connect with Application Board ! \n" " 1. Check if the board is connected and powered on. \n" " 2. Check if Application Board USB driver is installed. \n"
        //         " 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
        //     exit(result);
        // }

        // coines_set_shuttleboard_vdd_vddio_config(0, 0);
        delay_usec(100000);

        /* Bus configuration : I2C */
        if (intf == BMA5_I2C_INTF)
        {
            // printf("I2C Interface \n");

            dev_addr = BMA5_I2C_ADDRESS;
            bma5->bus_read = bma5_i2c_read;
            bma5->bus_write = bma5_i2c_write;
            bma5->intf = BMA5_I2C_INTF;

            // coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);
        }


        delay_usec(100000);

        // coines_set_shuttleboard_vdd_vddio_config(1800, 1800);

        delay_usec(100000);

        /* Holds the I2C device addr or SPI chip selection */
        bma5->intf_ptr = &dev_addr;

        /* Configure delay in microseconds */
        bma5->delay_us = bma5_delay_us;

        /* Assign context parameter */
        bma5->context = context;
    }
    else
    {
        rslt = BMA5_E_NULL_PTR;
    }

    return rslt;
}

void bma5_coines_deinit(void)
{
//     fflush(stdout);

//     coines_set_shuttleboard_vdd_vddio_config(0, 0);

//     coines_delay_msec(2000);

//     coines_soft_reset();

//     coines_delay_msec(100);

    // coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);
}

 uint16_t lengthtest;
int8_t mspm0c1103_I2C_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint16_t length){
	//I2C read operation and return uint16_t value as per datasheet
	// Data should be in first byte received from I2C denotes 15:8 and 2nd byte from I2C denotes 7:0
	int8_t returnValue=0;
    uint16_t gRxLen = length;
    if(length == 6)
    {
	   gRxLen = 6;	// default
        // __BKPT(0);
    }

    uint8_t writemode[2] = {0,};

    // mspm0c1103_I2C_write(dev_addr, reg_addr,NULL,length);
    mspm0c1103_I2C_write(dev_addr, reg_addr,NULL,0);
   
    /* Send a read request to Target */
    DL_I2C_startControllerTransfer(I2C_INST, dev_addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, gRxLen);



    for (uint8_t i = 0; i < gRxLen; i++) {
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_INST))
            ;
        reg_data[i] = DL_I2C_receiveControllerData(I2C_INST);
    }

	// returnValue = gRxPacket[0] << 8 | gRxPacket[1];
	return returnValue;

}

int8_t mspm0c1103_I2C_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length){
	//I2C write operation to the denoted register
	// Data should be in first byte sent on I2C denotes 15:8 and 2nd byte sent to I2C denotes 7:0
	uint32_t gTxLen, gTxCount;
	// uint16_t length;
    int8_t status = 0;

	if(length == 0)
	{
		length = 1;		// for read mode
        //  __BKPT(0);
	}
	else
	{
		length = 2;
	}

	uint8_t txBuffer[1 + length];
    // uint8_t txBuffer[length];
	
	txBuffer[0] = reg_addr;	// First byte is register address
	txBuffer[1] = reg_data[0];
	// txBuffer[2] = reg_data[1];
    // txBuffer[3] = reg_data[2];
    // txBuffer[4] = reg_data[3];
    
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
    DL_I2C_startControllerTransfer(I2C_INST, dev_addr,
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

    return status;
}

void delay_usec(uint32_t delay)
{
    uint32_t i;
    for(i=0; i<delay; i++)
    {
        delay_cycles(24);   // 1usec    // 24MHz 
    }
}