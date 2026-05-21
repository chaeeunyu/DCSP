#ifndef _MACRO_H
#define _MACRO_H

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <math.h>
#include <windows.h>
#include <direct.h>
#include <string.h>


// basic macro
#define		DAQ_DEV         "Dev8"
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
#define K_LIN				    (double)(8.2988)        // <---- MODIFY!!
#define COEFF_CW_A              (double)(-3.5463)        // <---- MODIFY!!
#define COEFF_CW_B              (double)(37.8204)        // <---- MODIFY!!
#define COEFF_CW_C              (double)(-77.1401)        // <---- MODIFY!!
#define COEFF_CCW_A             (double)(2.9428)        // <---- MODIFY!!
#define COEFF_CCW_B             (double)(3.9996)        // <---- MODIFY!!
#define COEFF_CCW_C             (double)(-23.4824)        // <---- MODIFY!!
#define DEAD_THRESH				(double)(0.5)

// motor sweep
#define HOLD_TIME           (double)(4.0)
#define N_HOLD              (int)(HOLD_TIME * SAMPLING_FREQ + 100)  
#define N_STEPS_MAX         200


#define TRI_AMP             (double)(1.0)
#define TRI_PERIOD          (double)(40.0)
#define TRI_CYCLES          (int)(5)
#define TRI_T_TOTAL         (double)(TRI_PERIOD * TRI_CYCLES)       
#define TRI_N_MAX           (int)(TRI_T_TOTAL * SAMPLING_FREQ + 200) 


#define SINE_AMP            (double)(1.0)
#define SINE_FREQ           (double)(0.025)
#define SINE_PERIOD         (double)(1.0 / SINE_FREQ)
#define SINE_CYCLES         (int)(5)
#define SINE_T_TOTAL        (double)(SINE_PERIOD * SINE_CYCLES)      
#define SINE_N_MAX          (int)(SINE_T_TOTAL * SAMPLING_FREQ + 200)
#define SINE_CMD(t)         (SINE_AMP * sin(2.0 * UNIT_PI * SINE_FREQ * (t)))

#define MODE_SINE  (0)
#define MODE_TRI   (1)

// run bode
#define BODE_SINE_AMP       (double)(0.45)
#define N_FREQS             (20)
#define N_SKIP_CYCLES       (int)(1)
#define N_CYCLES			(int)(5)

#define BODE_N_MAX          (int)(12000)

#define     BUF_SIZE        (int)(TRI_T_TOTAL * SAMPLING_FREQ + 200)


#endif