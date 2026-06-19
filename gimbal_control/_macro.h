#ifndef _MACRO_H
#define _MACRO_H

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <math.h>
#include <windows.h>
#include <direct.h>
#include <string.h>
#include <conio.h>
#include <time.h> //  for saving files -> !!! OUTSIDE do-while loop !!!


// basic macro
#define      DAQ_DEV			"Dev8"
#define      NEUTRAL			(float64)   (2.5)
#define      ON					(float64)   (5.0)
#define      OFF				(float64)   (0.0)
#define     SAMPLING_FREQ		(double)    (200.0)
#define     SAMPLING_TIME		(double)    (1.0 / SAMPLING_FREQ)
#define     UNIT_PI			    (double)    (3.14159265358979)
#define     K_GIMBAL			(double)    (1000.0 / 0.67 * UNIT_PI / 180.0)
#define     N_BIAS				(int)       (200)
#define		RECORD_TIME			(double)	(5.0)
#define		STABILIZATION_TIME	(double)	(10.0)

#define     READ_DATA(arr)			DAQmxReadAnalogF64(g_taskAI, 1, 10.0, DAQmx_Val_GroupByChannel, (arr), 3, &sampsPerChanRead, NULL)

// switch macro
#define     EXIT                (0)
#define     VOLTAGE_SWEEP       (1)
#define     FREQ_SWEEP          (2)
#define     SINE_VALIDATION     (3)
#define     TRI_VALIDATION      (4)
#define     STATIC_VALIDATION   (5)
#define     STEP_RESPONSE       (6)
#define     POT_POSITIONING     (7)
#define     POT_DATARECORD      (8)
#define     DESIGNATION         (9)
#define     STABILIZATION       (10)



// CW  4차 다항식: Vc = f(omega_deg)  <---- MODIFY after MATLAB polyfit 
#define CW_COEFF4   (double)(-2.123276e-13)
#define CW_COEFF3   (double)(+9.891086e-10)
#define CW_COEFF2   (double)(-5.800253e-07)
#define CW_COEFF1   (double)(+1.225497e-03)
#define CW_COEFF0   (double)(+2.722391e+00)

// CCW 
#define CCW_COEFF4  (double)(+1.174692e-12)
#define CCW_COEFF3  (double)(+3.376534e-09)
#define CCW_COEFF2  (double)(+2.532918e-06)
#define CCW_COEFF1  (double)(+1.886410e-03)
#define CCW_COEFF0  (double)( +2.308813e+00 )
#define WC_DZ       (double)(20.0)       // deadzibe [deg/s]
#define WC_SAT      (double)(1400.0)     // saturation [deg/s]

// motor sweep
#define HOLD_TIME           (double)(4.0)
#define N_HOLD              (int)(HOLD_TIME * SAMPLING_FREQ + 100)  
#define N_STEPS_MAX         200



#define TRI_AMP_DEGS        (double)(400.0)   // 진폭 [deg/s]  <---- MODIFY 
#define TRI_PERIOD          (double)(40)
#define TRI_CYCLES          (int)(5)
#define TRI_T_TOTAL         (double)(TRI_PERIOD * TRI_CYCLES)       
#define TRI_N_MAX           (int)(TRI_T_TOTAL * SAMPLING_FREQ + 200) 

#define SINE_AMP_DEGS       (double)(400.0)   
#define SINE_FREQ           (double)(0.025)
#define SINE_PERIOD         (double)(1.0 / SINE_FREQ)
#define SINE_CYCLES         (int)(5)
#define SINE_T_TOTAL        (double)(SINE_PERIOD * SINE_CYCLES)      
#define SINE_N_MAX          (int)(SINE_T_TOTAL * SAMPLING_FREQ + 200)
#define SINE_CMD_DEGS(t)    (SINE_AMP_DEGS * sin(2.0 * UNIT_PI * SINE_FREQ * (t)))  // [deg/s]

#define BUF_SIZE         (int)(TRI_T_TOTAL * SAMPLING_FREQ + 200)

#define MODE_SINE  (0)
#define MODE_TRI   (1)



#define BODE_SINE_AMP_DEGS  (double)(400.0)   // 진폭 [deg/s]  <---- MODIFY 
#define N_FREQS             (50)
#define N_SKIP_CYCLES       (int)(1)
#define N_CYCLES         (int)(5)

#define BODE_N_MAX          (int)(12000)


#define STATIC_AVG_TIME     (double)(2.0)
#define STATIC_AVG_N        (int)(STATIC_AVG_TIME * SAMPLING_FREQ)  // 400
#define STATIC_N_STEPS      (int)(51)
#define STATIC_CMD_MAX_DEGS (double)(1200.0)   // 최대 명령값 [deg/s]  <---- MODIFY 
#define STATIC_CMD_STEP     (double)(50.0)     // 명령 간격   [deg/s]  <---- MODIFY 


#define STEP_INPUT_DEGS         (double)(500.0)  // 스텝 명령값 [deg/s]  <---- MODIFY 
#define STEP_SETTLE_TIME        (double)(2.0)     

// potentiometer positioning
#define SPECIAL_KEY (224)
#define RIGHT_KEY   (75)
#define LEFT_KEY   (77)
#define EPS         (1.0)   // <--- MODIFY!!!



// Designation Loop control (PD Position Controller)
// MATLAB: Kp = Wc^2/Km, Kd = (2*Zc*Wc - Pm)/Km
// Wc = 37.0, Zc = 0.7, Km = 9.993, Pm = 10.87
#define KP              (double)(53.800738007380070)   // [1/s]  <--- VERIFY  
#define KD              (double)(1.752029520295203)    // [-]    <--- VERIFY  

// ── Potentiometer Calibration ─────────────────────────────────
// Vpot [V] - Vpot_ref [V] = K_POT * psi [deg]
#define K_POT               (double)(68.07352)   //  [deg/V] MODIFY after MATLAB polyfit 

// ── Designation Loop Timing & Logging ─────────────────────────
#define LOOP_SETTLE_TIME     (double)(1.5)      // pot 초기값 평균화 시간 [s] 
//#define LOOP_N_MAX           (int)(RECORD_TIME * SAMPLING_FREQ + 200)
#define PSI_NEUTRAL         (double)(2.511)		// [V]


// Stabilization Loop control (PI Controller)
#define KP_STB			(double)(1.0288)		// [-]
#define KI_STB			(double)(29.5203)	// [1/s]

// 단위 변환
#define RAD2DEG             (double)(180.0 / UNIT_PI)
#define DEG2RAD            (double)(UNIT_PI/180.0)


TaskHandle g_taskAI = 0;
TaskHandle g_taskAO = 0;
int32 sampsPerChanRead;
int32 sampsPerChanWritten;


// time 관련 변수들 structure로 묶으려다가 갯수가 애매해서 걍 놔뒀어요
// Vcmd, Vc, Vpot, omega, ... 이런애들도 어디는 쓰고 어디는 안쓰는데 있어서 걍 놔둠
// basic variable
double time_init = 0.0;
double time_elapsed = 0.0;
double Vg_offset = 0.0;
double Vcmd = 0.0;          /* 이제 [deg/s] 단위로 사용 (파형 명령값) */
double Vc = 0.0;
double Vg = 0.0;
double Vpot = 0.0;
double omega = 0.0;
double omega_target = 0.0;  /* InverseMap 내부에서 [rad/s]로 저장됨 */
double disturbance = 0.0;
double readArr[3] = { 0.0 };    // ai0, ai2, ai3
double writeArr[2] = { 0.0 };   // ao0, ao1, motor_power
int count = 0;
int savecount = 0;      // for saving files

/* Sweep (reused per step) */
double voltSeq[N_STEPS_MAX] = { 0 };

// 버퍼도 structure 만들려다가 걍 놔둠. 한개가지고 돌려쓰면 메모리 절약될거같아요
/* Triangle / Sine shared (size = TRI_N_MAX) */
double buftime[BUF_SIZE];
double bufVcmd[BUF_SIZE];   /* RunWaveVerify/Bode/Static/Step에서는 [deg/s] 저장 */
double bufVc[BUF_SIZE];
double bufVg[BUF_SIZE];
double bufVpot[BUF_SIZE];
double bufomega[BUF_SIZE];
double buf_OmegaCmd[BUF_SIZE];
double bufomega_target[BUF_SIZE];
double buf_disturbance[BUF_SIZE];


/* Bode results */
double bode_time[BODE_N_MAX];
double bode_Vcmd[BODE_N_MAX];   /* [deg/s] */
double bode_Vc[BODE_N_MAX];
double bode_Vg[BODE_N_MAX];
double bode_Vpot[BODE_N_MAX];
double bode_omega[BODE_N_MAX];
double bode_omega_target[BODE_N_MAX];

double bode_result_freq[N_FREQS];
double bode_result_gain[N_FREQS];
double bode_result_phase[N_FREQS];

double freq_step[N_FREQS] = { 0.0 };


/*Function Declarations*/
double GetWindowTime(void);
void   GetTimestampString(char* buf, size_t bufSize);
void   memorySet(void);
void   memorySet_bode(void);
void   BusyWait_ms(double ms);
void   WaitNextSample(void);
int    IsEmergencyStop(void);

void   DAQ_Init(void);
void   DAQ_Cleanup(void);
void   DAQ_ReadSample(void);
void   motor_power(float64 onoff, double voltage);

double CalculateGyroBias(int nSamples);
int    BuildVoltageSequence(void);
void   RunSweep(void);
double InverseMap(double omega_c_deg);   /* 변경: Vcmd[V] → omega_c[deg/s] 입력 */
double Triangle_cmd(double t);           /* 반환값: [deg/s] */
void   RunWaveVerify(int mode);
void   RunStaticVerify(void);

//void TEST_FREQS(void);
void   RunBode(void);
void   RunStepResponse(void);

// potentiometer positioning
void pot_positioning(void);
void RecordPotData(void);

// Loop Control
void RunDesignation(void);      // PD
void RunStabilization(void);    // PI



#endif