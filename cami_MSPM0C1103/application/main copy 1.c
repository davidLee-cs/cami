/*
 main.c
 
    2025.02.21
    - 보쉬 bma530 센서로 교체 작업 완료

    2025. 04.12 / 
     - MSPM0L1104 로 교체
     - 메모리 부족

 */

 #include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"
#include "opt3007_registers.h"
#include "opt3007_functions.h"
#include "opt3007_hostControl.h"
#include "math.h"
#include "accelerometer.h"



#define PI 3.14159265


#define OPTTIME_19MIN  (50*60*10)  // (20ms x 50 = 1sec) = 60초 x 10 = 10분 
#define OPTTIME_1MIN   (50*60)  // (20ms x 50 = 1sec) = 60초
#define OPTTIME_10SEC  (50*10)  // (20ms x 50 = 1sec) * 10 = 10초

#define DELAY (24000000/1000000)



float z_samples[SAMPLE_SIZE];
float fft_mag[SAMPLE_SIZE];

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

ti_opt3007_registers devReg;

bool bflag;
uint16_t optid;
uint16_t mxc1d;
double optlux;
uint16_t opt300checkCnt;
uint16_t fishcheckCnt;
volatile bool gTogglePolicy;
volatile bool gToggleLed;
volatile bool gFish_Red;
volatile bool gLED_On;

        // 샘플 수집
uint16_t samCnt=0;    

// 가속도 데이터 구조체
typedef struct {
    double x;
    double y;
    double z;
} AccelerationData;

// 이동 평균 필터 파라미터
#define MOVING_AVERAGE_WINDOW_SIZE 5
AccelerationData accel_history[MOVING_AVERAGE_WINDOW_SIZE];
int history_index = 0;

// 입질 감지를 위한 임계값 (조정 필요)
#define ACCELERATION_THRESHOLD 1.5 // 필터 적용 후 임계값 조정 필요
#define SIGNIFICANT_MOVEMENT_DURATION 5

// 고역 통과 필터 파라미터 (간단한 차분 형태)
#define HIGH_PASS_FILTER_FACTOR 0.8 // 0에 가까울수록 저주파 성분 제거 강도 증가

AccelerationData current_accel;
AccelerationData previous_filtered_accel = {0.0, 0.0, 0.0};
int significant_movement_count = 0;

// 필터링된 가속도 값 계산
AccelerationData apply_filters(AccelerationData current) {
    AccelerationData filtered;
    double sum_x = 0, sum_y = 0, sum_z = 0;

    // 이동 평균 필터 적용
    accel_history[history_index] = current;
    history_index = (history_index + 1) % MOVING_AVERAGE_WINDOW_SIZE;

    for (int i = 0; i < MOVING_AVERAGE_WINDOW_SIZE; i++) {
        sum_x += accel_history[i].x;
        sum_y += accel_history[i].y;
        sum_z += accel_history[i].z;
    }
    filtered.x = sum_x / MOVING_AVERAGE_WINDOW_SIZE;
    filtered.y = sum_y / MOVING_AVERAGE_WINDOW_SIZE;
    filtered.z = sum_z / MOVING_AVERAGE_WINDOW_SIZE;

    // 간단한 고역 통과 필터 적용 (현재 값 - 이전 필터링된 값 * 감쇠 계수)
    static AccelerationData previous_filtered = {0.0, 0.0, 0.0};
    AccelerationData high_passed;
    high_passed.x = filtered.x - previous_filtered.x * HIGH_PASS_FILTER_FACTOR;
    high_passed.y = filtered.y - previous_filtered.y * HIGH_PASS_FILTER_FACTOR;
    high_passed.z = filtered.z - previous_filtered.z * HIGH_PASS_FILTER_FACTOR;
    previous_filtered = filtered; // 현재 필터링된 값을 이전 값으로 저장

    return high_passed;
}


int main(){

    int i;


    // history 배열 초기화
    for (int i = 0; i < MOVING_AVERAGE_WINDOW_SIZE; i++) {
        accel_history[i] = (AccelerationData){0.0, 0.0, 0.0};
    }

    SYSCFG_DL_init();

    /* Set LED to indicate start of transfer */
    DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN_0_PIN);
    DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);
    // DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);
    delay_cycles(DELAY);

    gTogglePolicy = false;

    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    // NVIC_EnableIRQ(TIMER_1_INST_INT_IRQN);

    NVIC_EnableIRQ(I2C_INST_INT_IRQN);
    DL_SYSCTL_disableSleepOnExit();

    gI2cControllerStatus = I2C_STATUS_IDLE;

    ti_opt3007_registers devReg;
 
    bma530Accel_init();
	ti_opt3007_assignRegistermap(&devReg);
    ti_opt3007_setRn(&devReg);	

    DL_TimerG_startCounter(TIMER_0_INST);
    // DL_TimerG_startCounter(TIMER_1_INST);
    opt300checkCnt = OPTTIME_1MIN - 1;  // 첫번째 밝기 측정위해 설정
    gTogglePolicy = false;

    while(1){

        while (false == gTogglePolicy) {
            __WFE();
        }

        gTogglePolicy = false;
        DL_SYSCTL_setPowerPolicySTANDBY0();
        // DL_TimerG_stopCounter(TIMER_1_INST);

        while (false == gTogglePolicy) {
            __WFE();
        }

        gTogglePolicy = false;
        DL_SYSCTL_setPowerPolicyRUN0SLEEP0();  
        // DL_TimerG_startCounter(TIMER_1_INST);   

#if 1
        // if(opt300checkCnt++ >= OPTTIME_1MIN)
        {            
            ti_opt3007_setSensorSingleShot(&devReg);        // 싱글샷 모드에서 주기적으로 설정한 후 밝기 측정 함.
            optlux = ti_opt3007_readLux();

            if(opt300checkCnt >= OPTTIME_1MIN + 5){
                if(optlux < 10.0L) {
                    gLED_On = true;
                }
                else {
                    gLED_On = false;
                }

                opt300checkCnt = 0;
            }
        }
#endif
        bma530_readAccel();
        
        // 필터 적용
        AccelerationData filtered_accel = apply_filters(current_accel);

        // 가속도 변화량 계산 (필터링된 값 기준)
        double delta_x = fabs(filtered_accel.x - previous_filtered_accel.x);
        double delta_y = fabs(filtered_accel.y - previous_filtered_accel.y);
        double delta_z = fabs(filtered_accel.z - previous_filtered_accel.z);
        double delta_magnitude = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);

        if (delta_magnitude > ACCELERATION_THRESHOLD) {
            significant_movement_count++;
            if (significant_movement_count >= SIGNIFICANT_MOVEMENT_DURATION) {
                significant_movement_count = 0;
            }
        } else {
            significant_movement_count = 0;
        }

        previous_filtered_accel = filtered_accel;
    
        // samCnt++;
        // if(gFish_Red){

        //     if(fishcheckCnt++ >= 250){ 
        //         fishcheckCnt = 0;
        //         gFish_Red = false;
        //     }

        //     if(gLED_On){
        //         DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN_0_PIN);
        //     }
        //     DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);
        // }
        // else {
        //     if(gLED_On){
        //         DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);
        //     }
        //     DL_GPIO_clearPins(LED_RED_PORT, LED_RED_PIN_0_PIN);

        // }

        // delay_cycles(DELAY);

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


/**
 * STANDBY0 Clock, runs all the time at the same frequency
 */
void TIMER_0_INST_IRQHandler(void)
{
    static uint32_t count = 5;
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMERG_IIDX_ZERO:
            if (count == 1) {
                gTogglePolicy = true;
                count         = 1;
            } else {
                count--;
            }
            break;
        default:
            break;
    }
}

/**
 * SLEEP0 Clock, toggles LED in RUN0SLEEP0, does not toggles the LED
 * when running in STANDBY0
 */
// void TIMER_1_INST_IRQHandler(void)
// {
//     switch (DL_TimerG_getPendingInterrupt(TIMER_1_INST)) {
//         case DL_TIMERG_IIDX_ZERO:
//             // DL_GPIO_togglePins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);
            
//             break;
//         default:
//             break;
//     }
// }

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



