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
#define		STATIC_VALIDATION   (5)
#define		STEP_RESPONSE       (6)

// linearization 
#define K_LIN				    (double)(5.7412)        // <---- MODIFY!!
#define COEFF_CW_A              (double)(1.4501)        // <---- MODIFY!!
#define COEFF_CW_B              (double)( 9.1093 )        // <---- MODIFY!!
#define COEFF_CW_C              (double)(-35.1953)        // <---- MODIFY!!
#define COEFF_CCW_A             (double)(1.6606)        // <---- MODIFY!!
#define COEFF_CCW_B             (double)(12.3575)        // <---- MODIFY!!
#define COEFF_CCW_C             (double)(-36.6257)		//<---- MODIFY!!

#define VCMD_DEAD				(double)(0.05)
#define DEAD_THRESH				(double)(K_LIN * VCMD_DEAD)

#define VC_FIT_MIN				(double)( 1.5 )
#define VC_FIT_MAX				(double)( 3.5 )

#define VC_CW_BND   ((-COEFF_CW_B  + sqrt(COEFF_CW_B *COEFF_CW_B  - 4.0*COEFF_CW_A *(COEFF_CW_C  - K_LIN*VCMD_DEAD ))) / (2.0*COEFF_CW_A ))
#define VC_CCW_BND  ((-COEFF_CCW_B + sqrt(COEFF_CCW_B*COEFF_CCW_B - 4.0*COEFF_CCW_A*(COEFF_CCW_C - K_LIN*(-VCMD_DEAD)))) / (2.0*COEFF_CCW_A))


// motor sweep
#define HOLD_TIME           (double)(4.0)
#define N_HOLD              (int)(HOLD_TIME * SAMPLING_FREQ + 100)  
#define N_STEPS_MAX         200


// dynamic validation
#define TRI_AMP             (double)(1.5)
#define TRI_PERIOD          (double)(40)
#define TRI_CYCLES          (int)(5)
#define TRI_T_TOTAL         (double)(TRI_PERIOD * TRI_CYCLES)       
#define TRI_N_MAX           (int)(TRI_T_TOTAL * SAMPLING_FREQ + 200) 

#define SINE_AMP            (double)(1.5)
#define SINE_FREQ           (double)(0.025)
#define SINE_PERIOD         (double)(1.0 / SINE_FREQ)
#define SINE_CYCLES         (int)(5)
#define SINE_T_TOTAL        (double)(SINE_PERIOD * SINE_CYCLES)      
#define SINE_N_MAX          (int)(SINE_T_TOTAL * SAMPLING_FREQ + 200)
#define SINE_CMD(t)         (SINE_AMP * sin(2.0 * UNIT_PI * SINE_FREQ * (t)))

#define BUF_SIZE			(int)(TRI_T_TOTAL * SAMPLING_FREQ + 200)

#define MODE_SINE  (0)
#define MODE_TRI   (1)


// run bode
#define BODE_SINE_AMP       (double)(1.5)
#define N_FREQS             (50)
#define N_SKIP_CYCLES       (int)(1)
#define N_CYCLES			(int)(5)

#define BODE_N_MAX          (int)(12000)

// runstatcverify
#define STATIC_AVG_TIME     (double)(2.0)
#define STATIC_AVG_N        (int)(STATIC_AVG_TIME * SAMPLING_FREQ)  // 400
#define STATIC_N_STEPS      (int)(51)

// step response
#define STEP_INPUT              (double)(1.5)    // <---- MODIFY: Vcmd [V]
#define STEP_SETTLE_TIME        (double)(2.0)   
#define STEP_RECORD_TIME        (double)(5.0)    

#endif