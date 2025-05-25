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
// #include <cstdint>
// #include <cstdint>


// #define ADC_USE

#if ADC_USE
#define ADC12_0_INST                                                        ADC0
#define ADC12_0_INST_IRQHandler                                  ADC0_IRQHandler
#define ADC12_0_INST_INT_IRQN                                    (ADC0_INT_IRQn)
#define ADC12_0_ADCMEM_0                                      DL_ADC12_MEM_IDX_0
#define ADC12_0_ADCMEM_0_REF                   DL_ADC12_REFERENCE_VOLTAGE_INTREF
#define ADC12_0_ADCMEM_0_REF_VOLTAGE_V                                      2.50

/* clang-format off */
#define ADC12_BIT_RESOLUTION                (10)
#define ADC12_REF_VOLTAGE                   (2.5)
#define ADC12_SUPPLY_MONITOR_VOLTAGE        (2.6)
 #define ADC12_SUPPLY_MONITOR_VALUE         ( (1 << ADC12_BIT_RESOLUTION) * \
                                              (ADC12_SUPPLY_MONITOR_VOLTAGE / \
                                               (3 * ADC12_REF_VOLTAGE)))
/* clang-format on */

volatile bool gCheckADC;
volatile float gAdcResultVolts;
volatile int16_t gADCOffset;

void adcSet(void);
void batteryCheck(void);

#endif


/* Port definition for Pin Group LED_RED */
#define LED_RED_PORT                                                     (GPIOA)

/* Defines for PIN_0: GPIOA.27 with pinCMx 28 on package pin 1 */
#define LED_RED_PIN_0_PIN                                       (DL_GPIO_PIN_27)
#define LED_RED_PIN_0_IOMUX                                      (IOMUX_PINCM28)
/* Port definition for Pin Group LED_GREEN */
#define LED_GREEN_PORT                                                   (GPIOA)

/* Defines for PIN_1: GPIOA.24 with pinCMx 25 on package pin 8 */
#define LED_GREEN_PIN_1_PIN                                     (DL_GPIO_PIN_24)
#define LED_GREEN_PIN_1_IOMUX                                    (IOMUX_PINCM25)

#define MODE_CHECK      1
#define NORMAL_CHEMI    2
#define SMART_CHEMI     3
#define SENSITIVE_MODE  4

#define PI 3.14159265
#define OPTTIME_19MIN  (10*6*10)  // (100ms x 10 x 6 = 60초 x 10 
#define OPTTIME_1MIN   (10*6)  // 100ms x 10 x 6 = 60sec 
#define OPTTIME_10SEC  (10*10)  // (100ms x 10  = 1sec) * 10 = 10초
#define OPTTIME_30SEC  (10*10*3)  // (100ms x 10  = 1sec) * 10 * 3 = 30초

#define DELAY (24000000/1000000)

#define LOW_DELTA_THRESHOLD       (5.0L)     // 실험치 측정 필요
#define NIGHT_THRESHOLD           (10.0L)    // 밤 실험치 측정 필요

// --- 필터링 파라미터 ---
#define EWMA_ALPHA                0.3f    // EWMA 계수 (0<α<1)
#define HPF_FACTOR                0.9f    // 1차 IIR 고역통과 필터 계수

// --- 적응 임계치 초기화 파라미터 ---
#define CALIBRATION_DURATION_MS   5000    // 캘리브레이션 기간 (ms)
#define CALIBRATION_INTERVAL_MS   100     // 캘리 측정 간격 (ms)
#define THRESHOLD_STD_FACTOR      3.0f    // (평균 + k*표준편차)

// --- 이벤트 지속시간 및 딜레이 ---
#define DETECTION_LOCKOUT_MS      500     // 한번 감지 후 재감지 금지 시간 (ms)


// 재캘리브레이션 주기 (예: 1분마다)
#define RECALIBRATION_INTERVAL_MS   60000  
// 재캘리 샘플링 기간 (예: 2초)
#define RECALIBRATION_DURATION_MS   2000   
#define RECALIBRATION_SAMPLE_MS     50     

static uint32_t last_recal_time = 0;
static int recalib_sample_cnt = 0;
static float recalib_accum[RECALIBRATION_DURATION_MS / RECALIBRATION_SAMPLE_MS];

//uint32_t gRxLen, gRxCount;
void I2C_INST_IRQHandler(void);

void ledGpioSet(void);

/* Data sent to the Target */
uint8_t gTxPacket[I2C_TX_MAX_PACKET_SIZE] = {0x00,};
/* Counters for TX length and bytes sent */
uint32_t gTxLen, gTxCount;

/* Data received from Target */
uint8_t gRxPacket[I2C_RX_MAX_PACKET_SIZE];
/* Counters for TX length and bytes sent */
uint32_t gRxLen, gRxCount;

//ti_opt3007_registers devReg;

bool bflag;
uint16_t optid;
uint16_t mxc1d;
double optlux;
double ledOn_lux;
double ledOff_lux;
uint16_t opt300checkCnt;
uint16_t optBrightCnt;
uint16_t optDarkCnt;
uint16_t fishcheckCnt;
uint16_t lowVoltCnt;
uint16_t fishEndCnt;
volatile bool gTogglePolicy;
static bool gFish_Red;
static bool gLED_On;
static bool gAllLedOff = false;
static bool gLowVoltage = false;
bool bUpstate = false;
bool bDownstate = true;
uint16_t upCnt_zAxis = 0;
uint16_t downCnt_zAxis = 0;
uint16_t modeCheckCnt = 0;

uint16_t testcnt[10];

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
int16_t history_index = 0;

// 입질 감지를 위한 임계값 (Z축, 조정 필요)
#define ACCELERATION_THRESHOLD_0_Z      0.01//1.0 // Z축 변화량 임계값 (조정 필요)
#define ACCELERATION_THRESHOLD_1_Z      0.007//1.0 // Z축 변화량 임계값 (조정 필요)
#define SIGNIFICANT_MOVEMENT_DURATION   3 // 연속된 움직임 감지 횟수 (조정 필요)

// 고역 통과 필터 파라미터 (간단한 차분 형태)
#define HIGH_PASS_FILTER_FACTOR 0.8 // 0에 가까울수록 저주파 성분 제거 강도 증가

AccelerationData current_accel;
AccelerationData previous_filtered_accel = {0.0, 0.0, 0.0};
int16_t significant_movement_count = 0;
double delta_z = 0.0;
double threshold_level_z = 0.0;


uint32_t gnr_pin;
uint32_t red_pin;
uint16_t adcResult;
int16_t  modeSel;

static float ewma_z = 0.0f;
static float hpf_z_prev = 0.0f;
static float calib_samples[CALIBRATION_DURATION_MS / CALIBRATION_INTERVAL_MS];
static int   calib_cnt = 0;
static float thresh_mean = 0.0f, thresh_std = 0.0f;
static uint32_t last_detect_time = 0;



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
    uint16_t led=5;
    uint16_t cnt;

    SYSCFG_DL_init();
    ledGpioSet();

    // /* Set LED to indicate start of transfer */    
    // DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);
    // DL_GPIO_clearPins(LED_RED_PORT, LED_RED_PIN_0_PIN);
    // delay_cycles(24000000);
    DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN_0_PIN);
    DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);

    gTogglePolicy = false;

    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    // NVIC_EnableIRQ(TIMER_1_INST_INT_IRQN);

    NVIC_EnableIRQ(I2C_INST_INT_IRQN);
    DL_SYSCTL_disableSleepOnExit();

    gI2cControllerStatus = I2C_STATUS_IDLE;

    ti_opt3007_registers devReg;
 	ti_opt3007_assignRegistermap(&devReg);
    ti_opt3007_setRn(&devReg);	
    ti_opt3007_setSensorContinuous(&devReg);
    ti_opt3007_setSensorConversionTime100mS(&devReg);
    delay_cycles(24000);

    bma530Accel_init();
    delay_cycles(24000);

    DL_TimerG_startCounter(TIMER_0_INST);
    // DL_TimerG_startCounter(TIMER_1_INST);
    opt300checkCnt = OPTTIME_19MIN - 1;  // 첫번째 밝기 측정위해 설정
    gTogglePolicy = false;

    // history 배열 초기화
    for (int i = 0; i < MOVING_AVERAGE_WINDOW_SIZE; i++) {
        accel_history[i] = (AccelerationData){0.0, 0.0, 0.0};
    }
    modeSel = MODE_CHECK;

    while(1){

        gTogglePolicy = false;
        DL_I2C_disablePower(I2C_INST);
        DL_SYSCTL_setPowerPolicySTANDBY0();
        // DL_TimerG_stopCounter(TIMER_1_INST);

        while (false == gTogglePolicy) {
            __WFE();
        }

        DL_SYSCTL_setPowerPolicyRUN0SLEEP0();  
        DL_I2C_enablePower(I2C_INST);
        // DL_TimerG_startCounter(TIMER_1_INST);   

        switch(modeSel)
        {

            case MODE_CHECK :

                bma530_readAccel();
                current_accel.x = cami_accel[0];
                current_accel.y = cami_accel[1];
                current_accel.z = cami_accel[2];

                // down
                if((current_accel.z > 1.8) && (bDownstate == false))
                {
                    DL_GPIO_clearPins(LED_RED_PORT, LED_RED_PIN_0_PIN);
                    DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);
                    downCnt_zAxis++;
                    bUpstate = false;
                    bDownstate = true;
                }

                // Up
                if((current_accel.z < -1.25) &&  (bUpstate == false))
                {
                    DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN_0_PIN);
                    DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);
                    upCnt_zAxis++;
                    bUpstate = true;
                    bDownstate = false;
                }

                if(modeCheckCnt++ > 40)     // 4초
                {
                    __BKPT(0);

                    if(downCnt_zAxis == 2)
                    {
                        DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN_0_PIN);
                        DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);

                        for(cnt=0; cnt<6; cnt++)
                        {
                            // DL_GPIO_togglePins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);
                            DL_GPIO_togglePins(LED_RED_PORT, LED_RED_PIN_0_PIN);
                            delay_cycles(12000000);
                        }

                        DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN_0_PIN);
                        DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);

                        threshold_level_z = ACCELERATION_THRESHOLD_1_Z;     // 민감

                        modeSel = SMART_CHEMI;
                    }
                    else if(downCnt_zAxis >= 3)
                    {
                        DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN_0_PIN);
                        DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);

                        for(cnt=0; cnt<6; cnt++)
                        {
                            DL_GPIO_togglePins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);
                            // DL_GPIO_togglePins(LED_RED_PORT, LED_RED_PIN_0_PIN);
                            delay_cycles(12000000);
                        }

                        DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN_0_PIN);
                        DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN); // 일반모드 상시 켜짐

                        modeSel = NORMAL_CHEMI;
                    }
                    else {
                        DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN_0_PIN);
                        DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);

                        for(cnt=0; cnt<6; cnt++)
                        {
                            DL_GPIO_togglePins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);
                            DL_GPIO_togglePins(LED_RED_PORT, LED_RED_PIN_0_PIN);
                            delay_cycles(12000000);
                        }

                        DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN_0_PIN);
                        DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);

                        threshold_level_z = ACCELERATION_THRESHOLD_0_Z;     // 보통

                        modeSel = SMART_CHEMI;
                    }
                }
                else{
                    modeSel = MODE_CHECK;
                }

                break;
            case NORMAL_CHEMI : 
                    DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN_0_PIN);
                    DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);

                    modeSel = NORMAL_CHEMI;

                break;
            case SMART_CHEMI :

                if(opt300checkCnt++ >= OPTTIME_10SEC)
                {   
                    ledOn_lux = ti_opt3007_readLux();       // led + 주변환경 밝기 

                    DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);   // GRN OFF
                    DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN_0_PIN);           // RED OFF

                    ledOff_lux = ti_opt3007_readLux();      // 주변밝기
                    optlux = (ledOn_lux > ledOff_lux) ? (ledOn_lux - ledOff_lux) : 100.0;

                    if(optlux < LOW_DELTA_THRESHOLD)
                    {
                        // 한번 설정되면 배터리 제거 전까지 변경 안됨
                        gLowVoltage = true;
                    }

                    if(ledOff_lux < NIGHT_THRESHOLD) {
                        optDarkCnt++;
                    }
                    else {
                        optBrightCnt++;
                    }

                    if(opt300checkCnt >= OPTTIME_10SEC + 5)
                    {
                        if(optDarkCnt >= 3) {
                            gLED_On = true;
                        }
                        else if (optBrightCnt >= 3){
                            gLED_On = false;
                        }

                        optBrightCnt = 0;
                        optDarkCnt = 0;
                        opt300checkCnt = 0;
                    }
                    else{
                        gLED_On = false;
                    }
                }

                // gLED_On = true;
                if(gLED_On == true){

                    bma530_readAccel();
                    current_accel.x = cami_accel[0];
                    current_accel.y = cami_accel[1];
                    current_accel.z = cami_accel[2];
                    
                    if(current_accel.z > -0.7)
                    {
                        if(fishEndCnt++ > OPTTIME_30SEC)
                        {
                            gAllLedOff = true;
                        }
                    }
                    else {

                        gAllLedOff = false;
                    }

                    // 필터 적용
                    AccelerationData filtered_accel = apply_filters(current_accel);

                    // Z축 가속도 변화량 계산 (필터링된 값 기준)
                    delta_z = fabs(filtered_accel.z - previous_filtered_accel.z);

                    if (delta_z > threshold_level_z) {
                        significant_movement_count++;
                        if (significant_movement_count >= SIGNIFICANT_MOVEMENT_DURATION) {
                            gFish_Red = true;
                            significant_movement_count = 0;
                        }
                    } else {
                        significant_movement_count = 0;
                    }

                    previous_filtered_accel = filtered_accel;

                }

                if(gAllLedOff == false){
                    if(gFish_Red == true){

                        if(fishcheckCnt++ >= 18){  // 100ms x 50 = 5sec
                            fishcheckCnt = 0;
                            gFish_Red = false;
                        }

                        if(gLED_On == true){
                            DL_GPIO_clearPins(LED_RED_PORT, LED_RED_PIN_0_PIN);     // RED ON
                            testcnt[0]++;
                        }
                        else {
                            DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN_0_PIN);     // RED OFF
                            testcnt[1]++;
                        }
                        DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);       //GRN OFF
                        testcnt[3]++;
                    }
                    else {
                        if(gLED_On == true){
                            if(gLowVoltage == true)
                            {
                                if(lowVoltCnt++ > 15){      // 1.5초 주기
                                    lowVoltCnt = 0;
                                    DL_GPIO_togglePins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN); // GRN ON
                                }
                            }
                            else {
                                DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN); // GRN ON
                            
                            }
                        }
                        else 
                        {
                            DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);   // GRN OFF
                            testcnt[5]++;
                        }
                        DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN_0_PIN);           // RED OFF

                    }
                }
                else {
                    DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN_0_PIN);
                    DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);
                }

                 modeSel = SMART_CHEMI;

                break;

            default:

                break;
                
        }
        
        gTogglePolicy = false;

    };

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
void TIMER_1_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_1_INST)) {
        case DL_TIMERG_IIDX_ZERO:
            // DL_GPIO_togglePins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);
            
            break;
        default:
            break;
    }
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


void ledGpioSet(void)
{
    // DL_GPIO_reset(GPIOA);
    // DL_GPIO_enablePower(GPIOA);
    // delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initDigitalOutput(LED_RED_PIN_0_IOMUX);
    DL_GPIO_initDigitalOutput(LED_GREEN_PIN_1_IOMUX);
    DL_GPIO_clearPins(GPIOA, LED_RED_PIN_0_PIN |
		LED_GREEN_PIN_1_PIN);
    DL_GPIO_enableOutput(GPIOA, LED_RED_PIN_0_PIN |
		LED_GREEN_PIN_1_PIN);
}

#ifdef ADC_USE
void batteryCheck(void)
{

    while (false == gCheckADC) {
        __WFE();
    }

    gCheckADC = false;

    /* Result in integer for efficient processing */
    adcResult = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);

    /* Apply calibrated ADC offset - workaround for ADC_ERR_06 */
    int16_t adcRaw = (int16_t) adcResult + gADCOffset;
    if (adcRaw < 0) {
        adcRaw = 0;
    }
    if (adcRaw > 4095) {
        adcRaw = 4095;
    }
    adcResult = (uint16_t) adcRaw;

    /* Result in float for simpler reading */
    gAdcResultVolts =
        (adcResult * ADC12_REF_VOLTAGE) / (1 << ADC12_BIT_RESOLUTION) * 3;

    if (gAdcResultVolts > ADC12_SUPPLY_MONITOR_VOLTAGE) {
        gLowVoltage = false;
    } else {
        gLowVoltage = true;
    }

}


void ADC12_0_INST_IRQHandler(void)
{
    switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST)) {
        case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
            gCheckADC = true;
            break;
        default:
            break;
    }
} 

/* ADC12_0 Initialization */
static const DL_ADC12_ClockConfig gADC12_0ClockConfig = {
    .clockSel       = DL_ADC12_CLOCK_ULPCLK,
    .divideRatio    = DL_ADC12_CLOCK_DIVIDE_8,
    .freqRange      = DL_ADC12_CLOCK_FREQ_RANGE_20_TO_24,
};

void adcSet(void)
{
    // DL_GPIO_reset(GPIOA);
    DL_ADC12_reset(ADC12_0_INST);
    DL_VREF_reset(VREF);

    // DL_GPIO_enablePower(GPIOA);
    DL_ADC12_enablePower(ADC12_0_INST);
    DL_VREF_enablePower(VREF);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_ADC12_setClockConfig(ADC12_0_INST, (DL_ADC12_ClockConfig *) &gADC12_0ClockConfig);
    DL_ADC12_initSingleSample(ADC12_0_INST,
        DL_ADC12_REPEAT_MODE_ENABLED, DL_ADC12_SAMPLING_SOURCE_AUTO, DL_ADC12_TRIG_SRC_SOFTWARE,
        DL_ADC12_SAMP_CONV_RES_10_BIT, DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(ADC12_0_INST, ADC12_0_ADCMEM_0,
        DL_ADC12_INPUT_CHAN_15, DL_ADC12_REFERENCE_VOLTAGE_INTREF, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setSampleTime0(ADC12_0_INST,375);



    /* Enable ADC12 interrupt */
    DL_ADC12_clearInterruptStatus(ADC12_0_INST,(DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED));
    DL_ADC12_enableInterrupt(ADC12_0_INST,(DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED));
    DL_ADC12_enableConversions(ADC12_0_INST);
}

static const DL_VREF_ClockConfig gVREFClockConfig = {
    .clockSel = DL_VREF_CLOCK_BUSCLK,
    .divideRatio = DL_VREF_CLOCK_DIVIDE_1,
};
static const DL_VREF_Config gVREFConfig = {
    .vrefEnable     = DL_VREF_ENABLE_ENABLE,
    .bufConfig      = DL_VREF_BUFCONFIG_OUTPUT_2_5V,
    .shModeEnable   = DL_VREF_SHMODE_DISABLE,
    .holdCycleCount = DL_VREF_HOLD_MIN,
    .shCycleCount   = DL_VREF_SH_MIN,
};

SYSCONFIG_WEAK void SYSCFG_DL_VREF_init(void) {
    DL_VREF_setClockConfig(VREF,
        (DL_VREF_ClockConfig *) &gVREFClockConfig);
    DL_VREF_configReference(VREF,
        (DL_VREF_Config *) &gVREFConfig);
}

#endif