#ifndef _MACRO_H
#define _MACRO_H

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <math.h>
#include <windows.h>
#include <direct.h>
#include <string.h>


// basic macro
#define		DAQ_DEV         "Dev3"
#define		NEUTRAL			(float64)	(2.5)
#define		ON				(float64)	(5.0)
#define		OFF				(float64)	(0.0)
#define     SAMPLING_FREQ   (double)    (200.0)
#define     SAMPLING_TIME   (double)    (1.0 / SAMPLING_FREQ)
#define     UNIT_PI         (double)    (3.14159265358979)
#define     K_GIMBAL        (double)    (1000.0 / 0.67 * UNIT_PI / 180.0)
#define     N_BIAS          (int)       (200)

#define		READ_DATA(arr)	DAQmxReadAnalogF64(g_taskAI, 1, 10.0, DAQmx_Val_GroupByChannel, (arr), 2, &sampsPerChanRead, NULL)

// switch macro
#define     EXIT                (0)
#define     VOLTAGE_SWEEP       (1)
#define     FREQ_SWEEP          (2)
#define     SINE_VALIDATION     (3)
#define     TRI_VALIDATION      (4)
#define		STATIC_VALIDATION   (5)
#define		STEP_RESPONSE       (6)

// ── 신규 추가 ──────────────────────────────────────────────
// K_LIN 제거: 입력이 Vcmd[V] 대신 omega_c[deg/s]로 변경되어 불필요

/* CW  4차 다항식: Vc = f(omega_deg)  <---- MODIFY after MATLAB polyfit */
#define CW_C4   (double)(+3.989784e-13)
#define CW_C3   (double)(+7.448897e-11)
#define CW_C2   (double)(-4.677239e-07)
#define CW_C1   (double)(+1.261769e-03)
#define CW_C0   (double)(+2.693027e+00)

/* CCW */
#define CCW_C4  (double)(+7.466100e-14)
#define CCW_C3  (double)(+1.136119e-09)
#define CCW_C2  (double)(+1.220581e-06)
#define CCW_C1  (double)(+1.514228e-03)
#define CCW_C0  (double)(+2.306548e+00)
#define WC_DZ       (double)(20.0)       /* 데드존 경계 [deg/s]  */
#define WC_SAT      (double)(1400.0)     /* 포화 속도   [deg/s]  */

// motor sweep
#define HOLD_TIME           (double)(4.0)
#define N_HOLD              (int)(HOLD_TIME * SAMPLING_FREQ + 100)  
#define N_STEPS_MAX         200



#define TRI_AMP_DEGS        (double)(400.0)   /* 진폭 [deg/s]  <---- MODIFY */
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
#define SINE_CMD_DEGS(t)    (SINE_AMP_DEGS * sin(2.0 * UNIT_PI * SINE_FREQ * (t)))  /* [deg/s] */

#define BUF_SIZE			(int)(TRI_T_TOTAL * SAMPLING_FREQ + 200)

#define MODE_SINE  (0)
#define MODE_TRI   (1)



#define BODE_SINE_AMP_DEGS  (double)(400.0)   /* 진폭 [deg/s]  <---- MODIFY */
#define N_FREQS             (50)
#define N_SKIP_CYCLES       (int)(1)
#define N_CYCLES			(int)(5)

#define BODE_N_MAX          (int)(12000)


#define STATIC_AVG_TIME     (double)(2.0)
#define STATIC_AVG_N        (int)(STATIC_AVG_TIME * SAMPLING_FREQ)  // 400
#define STATIC_N_STEPS      (int)(51)
#define STATIC_CMD_MAX_DEGS (double)(1200.0)   /* 최대 명령값 [deg/s]  <---- MODIFY */
#define STATIC_CMD_STEP     (double)(50.0)     /* 명령 간격   [deg/s]  <---- MODIFY */


#define STEP_INPUT_DEGS         (double)(500.0)  /* 스텝 명령값 [deg/s]  <---- MODIFY */
#define STEP_SETTLE_TIME        (double)(2.0)   
#define STEP_RECORD_TIME        (double)(5.0)    

#endif
