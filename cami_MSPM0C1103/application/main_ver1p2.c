/*
 main.c
 
    2025.02.21
    - 보쉬 bma530 센서로 교체 작업 완료

    2025. 04.12 / 
     - MSPM0L1104 로 교체
     - 메모리 부족

    2025.05.25
    - 자동 보정 기능
    - 필터 기능 개선

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
#define OPTTIME_20MIN  (10*10*120)  // 1200초
#define OPTTIME_1MIN   (10*6)  // 100ms x 10 x 6 = 6sec 
#define OPTTIME_10SEC  (10*10)  // (100ms x 10  = 1sec) * 10 x 60 = 600초 
#define OPTTIME_30SEC  (10*10*3)  // (100ms x 10  = 1sec) * 10 * 3 = 30초

#define DELAY (24000000/1000000)

#define LOW_DELTA_THRESHOLD       (14.0L)     // 실험치 측정 필요 2.68 V ->14 lux
#define NIGHT_THRESHOLD           (10.0L)    // 밤 실험치 측정 필요

// --- 필터링 파라미터 ---
#define EWMA_ALPHA                0.3f    // EWMA 계수 (0<α<1)
#define HPF_FACTOR                0.9f    // 1차 IIR 고역통과 필터 계수

// --- 적응 임계치 초기화 파라미터 ---
#define CALIBRATION_DURATION_MS   12    // 캘리브레이션 기간 , 5초 x 12 = 60 초
#define CALIBRATION_INTERVAL_MS   100     // 캘리 측정 간격 (ms)
#define THRESHOLD_STD_FACTOR      1.0 //3.0f    // (평균 + k*표준편차)

// --- 이벤트 지속시간 및 딜레이 ---
#define DETECTION_LOCKOUT_MS      500     // 한번 감지 후 재감지 금지 시간 (ms)


// 재캘리브레이션 주기 (예: 1분마다)
#define RECALIBRATION_INTERVAL_MS   60000  
// 재캘리 샘플링 기간 (예: 2초)
#define RECALIBRATION_DURATION_MS   6  // 30초    
#define RECALIBRATION_SAMPLE_MS     50     

static uint32_t last_recal_time = 0;
static int recalib_sample_cnt = 0;
static float recalib_accum[RECALIBRATION_DURATION_MS / RECALIBRATION_SAMPLE_MS];

//uint32_t gRxLen, gRxCount;
void I2C_INST_IRQHandler(void);

void ledGpioSet(void);
float high_pass_filter(float raw_z);


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
double ledOn_lux[10];
double ledOff_lux[10];
uint16_t opt300checkCnt;
uint16_t optMeasureCnt;
uint16_t optBrightCnt;
uint16_t optDarkCnt;
uint16_t fishcheckCnt;
uint16_t lowVoltCnt;
uint16_t fishEndCnt;
volatile bool gTogglePolicy;
bool gFish_Red;
static bool gLED_On;
bool gNowNight = true;
static bool gAllLedOff = false;
static bool gLowVoltage = false;
bool bUpstate = false;
bool bDownstate = true;
bool bfirstCal = true;
uint16_t upCnt_zAxis = 0;
uint16_t downCnt_zAxis = 0;
uint16_t modeCheckCnt = 0;

uint16_t testcnt[10];
uint32_t gCnt_100ms;
uint32_t gVerticalCnt_100ms;
uint16_t gCnt_firstCalibration;
uint16_t gCnt_ReCalibration;

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
#define ACCELERATION_THRESHOLD_0_Z      (0.09f)//1.0 // Z축 변화량 임계값 (조정 필요)
#define ACCELERATION_THRESHOLD_1_Z      (0.09f)//1.0 // Z축 변화량 임계값 (조정 필요)
#define SIGNIFICANT_MOVEMENT_DURATION   3 // 연속된 움직임 감지 횟수 (조정 필요)
#define SLOW_DROP_THRESHOLD             (0.02f)
#define SLOW_DROP_DURATION_COUNTS   3      // 200 ms → 2샘플, 300 ms → 3샘플
#define DETECTION_LOCKOUT_COUNTS    10      // 1000 ms → 10샘플


// 고역 통과 필터 파라미터 (간단한 차분 형태)
#define HIGH_PASS_FILTER_FACTOR 0.8 // 0에 가까울수록 저주파 성분 제거 강도 증가

AccelerationData current_accel;
AccelerationData previous_filtered_accel = {0.0, 0.0, 0.0};
int16_t significant_movement_count = 0;
double delta_z = 0.0;
float threshold_level_z = 0.0;
float previous_accel_z = 0.0;

uint32_t gnr_pin;
uint32_t red_pin;
uint16_t adcResult;
int16_t  modeSel;
uint16_t bSensitivityLevel = 1;

// 캘리브레이션 변수
static float ewma_z = 0.0f;
static float hpf_z_prev = 0.0f;
static float calib_samples[CALIBRATION_DURATION_MS / CALIBRATION_INTERVAL_MS];
static int   calib_cnt = 0;
static float thresh_mean = 0.0f, thresh_std = 0.0f;
static uint32_t last_detect_time = 0;

float raw_z;
float prev_raw_z;
float z_delta;
float prev_z_delta;
float zz_check;

// 추가 설정
#define VERTICAL_STABLE_DURATION_MS  50    // 2초  수직 상태가 연속 유지돼야 하는 시간
#define INITIAL_CALIB_DELAY_MS       600   //60초 수직 후 대기할 시간 (1분)

// 상태 정의
typedef enum {
    STATE_WAIT_VERTICAL,
    STATE_DELAY_BEFORE_CALIB,
    STATE_OPERATING
} SystemState;

// 전역 변수
static SystemState sys_state = STATE_WAIT_VERTICAL;
static uint32_t vertical_detect_time = 0;

bool en_test = false;
bool vertical;
bool cx;
bool cy;
bool cz;

static bool     slow_event_flag = false;
static int      slow_event_cnt = 0;
static int      lockout_cnt = 0;
static bool     in_lockout = false;
float low_freq;
float drop;


// 수직 상태 판정 함수
bool is_vertical_stable(void) {
    static uint32_t start = 0;
    static bool timing = false;
        
    // Z축이 중력가속도 ≈1g, X/Y축은 거의 0g 근처인지 확인
    const float G = -1.8f;
    const float TOL = 0.5f;

    cx = (fabs((float)current_accel.x) < TOL);
    cy = (fabs((float)current_accel.y) < TOL);
    cz = (fabs((float)current_accel.z - G) < TOL+0.3);

    vertical = cx && cy && cz;
    // vertical = (fabs((float)current_accel.x - G) < TOL) &&
    //                 (fabs((float)current_accel.y) < TOL) &&
    //                 (fabs((float)current_accel.z) < TOL);

    if(en_test)   __BKPT(0);
    // if (vertical) {
    //     if (!timing) {
    //         timing = true;
    //         gVerticalCnt_100ms = 0;
    //     } else if (gVerticalCnt_100ms >= VERTICAL_STABLE_DURATION_MS) {
    //         // 충분히 수직 상태가 유지됨
    //         timing = false;
    //         // gCnt_firstCalibration = 0;
    //         return true;
    //     }
    // } else {
    //     // 수직 판정 실패 시 타이머 리셋
    //     timing = false;
    // }

    if (!timing) {
        timing = true;
        gVerticalCnt_100ms = 0;
    } else if (gVerticalCnt_100ms >= VERTICAL_STABLE_DURATION_MS) {
        // 충분히 수직 상태가 유지됨
        timing = false;
        // gCnt_firstCalibration = 0;
        return true;
    }

    return false;
}

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

void calibrate_threshold(float z_axis) {

    if(++gCnt_firstCalibration < CALIBRATION_DURATION_MS) {
        // 가속도 읽기
        double z = current_accel.z;
        float delta = fabsf(high_pass_filter(z));

        // EWMA + HPF 적용 (하지만 캘리용으론 생략 가능)
        calib_samples[calib_cnt++] = (float)delta;
    }else {
        // 평균 계산
        for (int i = 0; i < calib_cnt; i++) thresh_mean += calib_samples[i];
        thresh_mean /= calib_cnt;
        // 표준편차 계산
        for (int i = 0; i < calib_cnt; i++)
            thresh_std += powf(calib_samples[i] - thresh_mean, 2);
        thresh_std = sqrtf(thresh_std / calib_cnt);
        
        // 최종 임계치 = mean + k * std
        threshold_level_z = thresh_mean + THRESHOLD_STD_FACTOR * thresh_std;
        if(bSensitivityLevel == 2) threshold_level_z = threshold_level_z * 0.7;
        gCnt_ReCalibration = 0;
        recalib_sample_cnt = 0;
        bfirstCal = false;  // 첫 cal 끝
        gCnt_firstCalibration = 0;

    }
}

void recalibrate_threshold(float z_axis) {
    // 짧게 샘플링하면서 Z축 값 수집
    if(++gCnt_ReCalibration < RECALIBRATION_DURATION_MS) {

        float redelta = fabsf(high_pass_filter((float)current_accel.z));

        recalib_accum[recalib_sample_cnt++] = redelta;
    }else {
        // 평균·표준편차 계산
        float sum = 0, sum2 = 0;
        for (int i = 0; i < recalib_sample_cnt; i++) {
            sum  += recalib_accum[i];
            sum2 += recalib_accum[i] * recalib_accum[i];
        }
        float mean = sum / recalib_sample_cnt;
        float var  = (sum2 / recalib_sample_cnt) - (mean * mean);
        float std  = sqrtf(var > 0 ? var : 0);

        gCnt_ReCalibration = 0; // 다음에 다시 보정 위해 초기화
        recalib_sample_cnt = 0; // 다음에 다시 보정 위해 초기화
        threshold_level_z = mean + THRESHOLD_STD_FACTOR * std;
        if(bSensitivityLevel == 2) threshold_level_z = threshold_level_z * 0.7;
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

// Z축만 예시. X/Y도 같은 구조로 확장 가능
float high_pass_filter(float raw_z) {
    // EWMA 로 저주파 
    ewma_z = EWMA_ALPHA * raw_z + (1 - EWMA_ALPHA) * ewma_z;
    low_freq = ewma_z;
    
    // 1차 IIR 고역통과
    float hpf_z = raw_z - low_freq + HPF_FACTOR * hpf_z_prev;
    hpf_z_prev = hpf_z;
    return hpf_z;
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

// opt3007과 bma530 순서 주의!!!
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
    opt300checkCnt = 0;
    gTogglePolicy = false;

    // history 배열 초기화
    for (int i = 0; i < MOVING_AVERAGE_WINDOW_SIZE; i++) {
        accel_history[i] = (AccelerationData){0.0, 0.0, 0.0};
    }
    modeSel = MODE_CHECK;
    while(1){
    
#if 1
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
                    // __BKPT(0);

                    if(downCnt_zAxis == 2)  // 민감 모드
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

                        threshold_level_z = ACCELERATION_THRESHOLD_1_Z * 0.7;     // 민감

                        bSensitivityLevel = 2;
                        modeSel = SMART_CHEMI;
                    }
                    else if(downCnt_zAxis >= 3) // 일반캐미 모드
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
                    else {  // 스마트캐미 모드
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
                        bSensitivityLevel = 1;
                        modeSel = SMART_CHEMI;
                    }
                }
                else{
                    modeSel = MODE_CHECK;
                }

                break;
            case NORMAL_CHEMI : 
                    // 일반 캐미 모드는 배터리 끝날때까지 LED ON 시킴.
                    DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN_0_PIN);
                    DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);

                    modeSel = NORMAL_CHEMI;

                break;
            case SMART_CHEMI :

                // 조도 몇 배터리 모니터링 기능
                if(opt300checkCnt++ >= OPTTIME_30SEC){   
                    ledOn_lux[optMeasureCnt] = ti_opt3007_readLux();       // led + 주변환경 밝기 

                    DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN);   // GRN OFF
                    DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN_0_PIN);           // RED OFF
                    delay_cycles(12000000);
                    ledOff_lux[optMeasureCnt] = ti_opt3007_readLux();      // 주변밝기
                    optlux = (ledOn_lux[0] > ledOff_lux[0]) ? (ledOn_lux[0] - ledOff_lux[0]) : 100.0;

                    if(gNowNight == true) {
                        DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN_1_PIN); // GRN ON
                    }

                    if(optlux < LOW_DELTA_THRESHOLD)
                    {
                        // 한번 설정되면 배터리 제거 전까지 변경 안됨
                        gLowVoltage = true;
                    }

                    if(ledOff_lux[optMeasureCnt] < NIGHT_THRESHOLD) {
                        optDarkCnt++;
                    }
                    else {
                        optBrightCnt++;
                    }

                    optMeasureCnt++;      
                    
                    if(opt300checkCnt >= OPTTIME_30SEC + 5)
                    {

                        if(optDarkCnt >= 3) {
                            gNowNight = true;
                            optMeasureCnt = 0;
                        }
                        else if (optBrightCnt >= 3){
                            gNowNight = false;
                            optMeasureCnt = 0;
                        }

                        optBrightCnt = 0;
                        optDarkCnt = 0;
                        opt300checkCnt = 0;
                    }
                    else{
                        gNowNight = false;
                    }                                  
                }

                
                // gNowNight = true;
                // gLowVoltage = false;
                // 입질 기능
                if(gNowNight == true){    // 즉 아래 기능은 밤에만 작동하도록 함.

                    bma530_readAccel();
                    current_accel.x = cami_accel[0];
                    current_accel.y = cami_accel[1];
                    current_accel.z = cami_accel[2];

 #if 1
                    raw_z = (float)current_accel.z;
                    z_delta = fabsf(high_pass_filter(raw_z));
                    zz_check = fabsf(prev_z_delta - z_delta);
                    bool fast_detected  = ( zz_check > threshold_level_z);

                    // 3) 느린 드리프트 감지 (연속 SLOW_DROP_DURATION_COUNTS샘플 유지)
                    bool slow_detected = false;
                    drop = fabsf(prev_raw_z - low_freq);
                    // drop = prev_raw_z - raw_z;

                    if (drop > SLOW_DROP_THRESHOLD) {
                        significant_movement_count++;
                        // 순간 민감도에 의한 오류 제거위해 카운터 적용
                        if (significant_movement_count >= SLOW_DROP_DURATION_COUNTS) {
                            // gFish_Red = true;
                            slow_detected = true;
                            significant_movement_count = 0;
                        }
                    } else {
                        significant_movement_count = 0;
                    }
                    
                // 4) 락아웃 카운터 처리
                    if (in_lockout) {
                        if (++lockout_cnt >= DETECTION_LOCKOUT_COUNTS) {
                            in_lockout = false;
                            lockout_cnt = 0;
                        }
                    }

                    // 5) 최종 입질 판정
                    if (!in_lockout && (fast_detected || slow_detected)) {
                        gFish_Red   = true;
                        in_lockout  = true;
                        lockout_cnt = 0;
                        // → 이곳에 입질 시 실행할 액션 추가
                    } 

                    prev_z_delta = z_delta;
                    prev_raw_z = low_freq;
                    // prev_raw_z = raw_z;

#else

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

#endif

#if 0
                    // 주기적으로 자동 보정 기능 
                    if(is_vertical_stable()){

                        calibrate_threshold(z_delta);

                        // if(bfirstCal){
                        //     calibrate_threshold(z_delta);
                        // }else{
                        //     recalibrate_threshold(z_delta);
                        // }
                    }else if(current_accel.z > -0.7) {
                        if(fishEndCnt++ > OPTTIME_30SEC)
                        {
                            // 기울어져 있어 사용. 대기중..
                            gAllLedOff = true;
                        }
                    }else {

                        gAllLedOff = false;
                    }

#endif                    
                }

                // led 관련 기능
                if(gAllLedOff == false){
                    if(gFish_Red == true){

                        if(fishcheckCnt++ >= 18){  // 1.8 sec
                            fishcheckCnt = 0;
                            gFish_Red = false;
                        }

                        if(gNowNight == true){
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
                        if(gNowNight == true){
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
                        fishcheckCnt = 0;
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
#endif
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
                gCnt_100ms++;
                gVerticalCnt_100ms++;
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

