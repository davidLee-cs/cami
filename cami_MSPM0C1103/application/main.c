/*
 * main.c
 *
 *  Created on: Sep 17, 2020
 *      Author: Karthik Rajagopal
 */
 #include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"
#include "opt3007_registers.h"
#include "opt3007_functions.h"
#include "opt3007_hostControl.h"
#include "mxc4005_registers.h"
#include "mxc4005_functions.h"
#include "mxc4005_hostControl.h"


//uint32_t gRxLen, gRxCount;
void I2C_INST_IRQHandler(void);

/* Data sent to the Target */
uint8_t gTxPacket[I2C_TX_MAX_PACKET_SIZE] = {0x00,};
/* Counters for TX length and bytes sent */
uint32_t gTxLen, gTxCount;

/* Data received from Target */
uint8_t gRxPacket[I2C_RX_MAX_PACKET_SIZE];
/* Counters for TX length and bytes sent */
uint32_t gRxLen, gRxCount;

#define DELAY (48000000)
bool bflag;

uint16_t optid;
uint16_t mxc1d;
double optlux;

int main(){

    int i;

    SYSCFG_DL_init();

    /* Set LED to indicate start of transfer */
    DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN_0_PIN);
    DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);
    // DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);
    delay_cycles(DELAY);

    NVIC_EnableIRQ(I2C_INST_INT_IRQN);
    DL_SYSCTL_disableSleepOnExit();

    gI2cControllerStatus = I2C_STATUS_IDLE;

//	printf("Testing code for OPT3007 C driver\n");
	ti_opt3007_registers devReg;
    mxc4005_registers mxc_devReg;
    
	ti_opt3007_assignRegistermap(&devReg);
    mxc4005_assignRegistermap(&mxc_devReg);

	// ti_opt3007_setSensorContinuous(&devReg);
    ti_opt3007_setSensorSingleShot(&devReg);
	ti_opt3007_setSensorConversionTime100mS(&devReg);
    // ti_opt3007_setRn(&devReg);	

    // uint16_t optid = ti_opt3007_readManufacturerID(&devReg);
    // uint16_t mxc1d = mxc4005_readManufacturerID(&mxc_devReg);
    // double optlux = ti_opt3007_readLux();
    // mxc4005_readLux();

    while(1){

        // if(bflag ^= 1)
        // {
        //     // DL_GPIO_clearPins(LED_RED_PORT, LED_RED_PIN_0_PIN);
        //     DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);

        //     // optid = ti_opt3007_readManufacturerID(&devReg);

        //     // mxc1d = mxc4005_readManufacturerID(&mxc_devReg);
        //     // mxc1d = mxc4005_readDeviceID(&mxc_devReg);
        //     mxc4005_readAccel();
        // }
        // else
        // {
        //     // DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN_0_PIN);
        //     DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);
        //     optlux = ti_opt3007_readLux();
        //     // mxc4005_readAccel();
        // }

        ti_opt3007_setSensorSingleShot(&devReg);        // 싱글샷 모드에서 주기적으로 설정한 후 밝기 측정 함.
        optlux = ti_opt3007_readLux();
        if(optlux < 100) {

            for(i=0; i<10; i++)
            {
                if(bflag ^= 1) {
                    DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN); // LED ON
                }
                else {                
                    DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);   // LED OFF
                }

                delay_cycles(DELAY/20);
            }
            DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);     // LED ON
                    // __BKPT(0);
        }

        mxc4005_readAccel();
        delay_cycles(DELAY);
    };

//	printf("Manufacturer ID:0x%02x DeviceID:0x%02x\n",ti_opt3007_readManufacturerID(&devReg),ti_opt3007_readDeviceID(&devReg));
//	printf("OPT3007 Reading:%g lux\n",ti_opt3007_readLux());
	/* Available functions to set the device
        void ti_opt3007_setSensorShutDown(ti_opt3007_registers* devReg);
        void ti_opt3007_setSensorSingleShot(ti_opt3007_registers* devReg);
        void ti_opt3007_setSensorContinuous(ti_opt3007_registers* devReg);
        void ti_opt3007_setSensorConversionTime100mS(ti_opt3007_registers* devReg);
        void ti_opt3007_setSensorConversionTime800mS(ti_opt3007_registers* devReg);
        double ti_opt3007_readLux(void);
        uint16_t ti_opt3007_readManufacturerID(ti_opt3007_registers* devReg);
        uint16_t ti_opt3007_readDeviceID(ti_opt3007_registers* devReg);

	For direct register writes to registers in the data sheet... for eg:
	If its desired to write register TH is 0x10.. here is the example...
        ti_opt3007_deviceRegister_write(&devReg->TH,0x10);
	if desired to read a register from data sheet for example TH
        readValue=ti_opt3007_deviceRegister_read(&devReg->TH);

*/

	return 0;
}

/* Interrupt handler for I2C */
void I2C_INST_IRQHandler(void) {
    switch (DL_I2C_getPendingInterrupt(I2C_INST)) {
        case DL_I2C_IIDX_CONTROLLER_RX_DONE:
            gI2cControllerStatus = I2C_STATUS_RX_COMPLETE;
            break;
        case DL_I2C_IIDX_CONTROLLER_TX_DONE:
            DL_I2C_disableInterrupt(I2C_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
            gI2cControllerStatus = I2C_STATUS_TX_COMPLETE;
            break;
        case DL_I2C_IIDX_CONTROLLER_RXFIFO_TRIGGER:
            gI2cControllerStatus = I2C_STATUS_RX_INPROGRESS;
            /* Receive all bytes from target */
            while (DL_I2C_isControllerRXFIFOEmpty(I2C_INST) != true) {
                if (gRxCount < gRxLen) {
                    gRxPacket[gRxCount++] = DL_I2C_receiveControllerData(I2C_INST);
                } else {
                    /* Ignore and remove from FIFO if the buffer is full */
                    DL_I2C_receiveControllerData(I2C_INST);
                }
            }
            break;
        case DL_I2C_IIDX_CONTROLLER_TXFIFO_TRIGGER:
            gI2cControllerStatus = I2C_STATUS_TX_INPROGRESS;
            /* Fill TX FIFO with next bytes to send */
            if (gTxCount < gTxLen) {
                gTxCount += DL_I2C_fillControllerTXFIFO(I2C_INST, &gTxPacket[gTxCount], gTxLen - gTxCount);
            }
            break;
        case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
        case DL_I2C_IIDX_CONTROLLER_NACK:
            if ((gI2cControllerStatus == I2C_STATUS_RX_STARTED) ||
                (gI2cControllerStatus == I2C_STATUS_TX_STARTED)) {
                /* NACK interrupt if I2C Target is disconnected */
                gI2cControllerStatus = I2C_STATUS_ERROR;
            }
        case DL_I2C_IIDX_CONTROLLER_RXFIFO_FULL:
        case DL_I2C_IIDX_CONTROLLER_TXFIFO_EMPTY:
        case DL_I2C_IIDX_CONTROLLER_START:
        case DL_I2C_IIDX_CONTROLLER_STOP:
        case DL_I2C_IIDX_CONTROLLER_EVENT1_DMA_DONE:
        case DL_I2C_IIDX_CONTROLLER_EVENT2_DMA_DONE:
        default:
            break;
    }
}
