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

#define		READ_DATA(arr)	DAQmxReadAnalogF64(g_taskAI, 1, 10.0, DAQmx_Val_GroupByChannel, (arr), 2, &sampsPerChanRead, NULL);

// switch macro
#define     EXIT                (0)
#define     VOLTAGE_SWEEP       (1)
#define     FREQ_SWEEP          (2)
#define     SINE_VALIDATION     (3)
#define     TRI_VALIDATION      (4)

// linearization --------------------> P_CW_A/B/C 이거 readibility 좋게 변수명 좀 바꿔주세요 -> 몇번째 자리 coeff인지 나타내는 변수이름이면 좋을듯
#define K_LIN				    (double)(   4.9596)        // <---- MODIFY!!
#define COEFF_CW_A              (double)( -0.0991 )        // <---- MODIFY!!
#define COEFF_CW_B              (double)( 16.4033 )        // <---- MODIFY!!
#define COEFF_CW_C              (double)(-43.7992)        // <---- MODIFY!!
#define COEFF_CCW_A             (double)( -1.5739)        // <---- MODIFY!!
#define COEFF_CCW_B             (double)(20.5965)        // <---- MODIFY!!
#define COEFF_CCW_C             (double)(-38.8920) //<---- MODIFY!!
//#define DEAD_THRESH				(double)(0.2)
#define VCMD_DEAD				(double)(0.05)
#define DEAD_THRESH				(double)(K_LIN * VCMD_DEAD)
//#define DEAD_ZONE_LINEAR	    (double)(8.8496)        //dead zone linareze voltage
#define VC_FIT_MIN				(double)( 1.5 )
#define VC_FIT_MAX				(double)( 3.5 )

#define VC_CW_BND   ((-COEFF_CW_B  + sqrt(COEFF_CW_B *COEFF_CW_B  - 4.0*COEFF_CW_A *(COEFF_CW_C  - K_LIN*VCMD_DEAD ))) / (2.0*COEFF_CW_A ))
#define VC_CCW_BND  ((-COEFF_CCW_B + sqrt(COEFF_CCW_B*COEFF_CCW_B - 4.0*COEFF_CCW_A*(COEFF_CCW_C - K_LIN*(-VCMD_DEAD)))) / (2.0*COEFF_CCW_A))

/* ============================================================
 *  상수 정의 (MATLAB WLS 1차 피팅 결과로 채울 것)
 * ============================================================ */
#define COEFF_CW_SLOPE    ( 14.0195 )   /* CW  기울기  [rad/s per V] */
#define COEFF_CW_INTCPT   ( -38.0255 )   /* CW  절편    [rad/s]       */
#define COEFF_CCW_SLOPE   ( 13.2231  )   /* CCW 기울기  [rad/s per V] */
#define COEFF_CCW_INTCPT  ( -30.2586 )   /* CCW 절편    [rad/s]       */

//#define K_LIN             ( 9.4588)   /* 선형화 이득 [(rad/s)/V]   */
//#define DEAD_THRESH       (0.1)                /* 데드존 omega 임계 [rad/s] */
//#define VCMD_DEAD         (DEAD_THRESH / K_LIN)/* 데드존 Vcmd 임계 [V]      */
//#define VC_DEAD_POS       (( 0.5 - COEFF_CW_INTCPT)  / COEFF_CW_SLOPE)   /* CW  데드존 경계 Vc */
//#define VC_DEAD_NEG       ((-0.5 - COEFF_CCW_INTCPT) / COEFF_CCW_SLOPE)  /* CCW 데드존 경계 Vc */

// motor sweep
#define HOLD_TIME           (double)(4.0)
#define N_HOLD              (int)(HOLD_TIME * SAMPLING_FREQ + 100)  
#define N_STEPS_MAX         200


#define TRI_AMP             (double)(2)
#define TRI_PERIOD          (double)(40)
#define TRI_CYCLES          (int)(3)
#define TRI_T_TOTAL         (double)(TRI_PERIOD * TRI_CYCLES)       
#define TRI_N_MAX           (int)(TRI_T_TOTAL * SAMPLING_FREQ + 200) 


#define SINE_AMP            (double)(2.0)
#define SINE_FREQ           (double)(0.025)
#define SINE_PERIOD         (double)(1.0 / SINE_FREQ)
#define SINE_CYCLES         (int)(10)
#define SINE_T_TOTAL        (double)(SINE_PERIOD * SINE_CYCLES)      
#define SINE_N_MAX          (int)(SINE_T_TOTAL * SAMPLING_FREQ + 200)
#define SINE_CMD(t)         (SINE_AMP * sin(2.0 * UNIT_PI * SINE_FREQ * (t)))

#define MODE_SINE  (0)
#define MODE_TRI   (1)

// run bode
#define BODE_SINE_AMP       (double)(2.0)
#define N_FREQS             (60)
#define N_SKIP_CYCLES       (int)(1)
#define N_CYCLES			(int)(5)

#define BODE_N_MAX          (int)(12000)

#define BUF_SIZE        (int)(TRI_T_TOTAL * SAMPLING_FREQ + 200)

//runstatcverify
#define STATIC_VERIFY       (5)
#define STATIC_AVG_TIME     (double)(2.0)
#define STATIC_AVG_N        (int)(STATIC_AVG_TIME * SAMPLING_FREQ)  // 400
#define STATIC_N_STEPS      (int)(51)

// step response
// step response
#define STEP_RESPONSE           (6)

#define STEP_INPUT              (double)(2.0)    // <---- MODIFY: 스텝 크기 Vcmd [V]
#define STEP_SETTLE_TIME        (double)(2.0)    // 스텝 인가 전 neutral 유지 시간 [s]
#define STEP_RECORD_TIME        (double)(5.0)    // 스텝 인가 후 기록 시간 [s]

#endif