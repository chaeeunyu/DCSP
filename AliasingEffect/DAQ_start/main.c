#define _CRT_SECURE_NO_WARNINGS  // Visual Studio에서 fopen 같은 함수 경고 제거

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <windows.h>
#include <time.h>
#include "NIDAQmx.h"        // NI DAQ 장비 제어용 라이브러리



// ------------------------- 현재 시간 -----------------------------------
double GetWindowTime(void)
{
    LARGE_INTEGER   liEndCounter, liFrequency;

    QueryPerformanceCounter(&liEndCounter);     // 현재 카운터값
    QueryPerformanceFrequency(&liFrequency);    // 카운터 주파수

    return(liEndCounter.QuadPart / (double)(liFrequency.QuadPart) * 1000.0);
}; // [ms]


// ------------------------------------------------------------
// 실험 설정 파라미터
// ------------------------------------------------------------
#define   FINAL_TIME      (double)(            5.0 )           //  신호 나오는 duration [s]
#define   SAMPLING_FREQ      (double)(               200 )      // 샘플링 주파수, f_s [Hz]
#define   SAMPLING_TIME     (double)( 1.0/SAMPLING_FREQ )       // 샘플링 주기 [s]
#define   N_STEP          (int)   ( FINAL_TIME*SAMPLING_FREQ )  // 총 샘플 개수
#define   UNIT_PI         (double)( 3.14159265358979  )

// ------------------------------------------------------------------------------------------
//    MAIN
// --------------------------------------

void main(void)
{
    FILE* pFile;

    // 시간 관련 변수 선언
    double      time_curr = 0.0;
    double      time_init = 0.0;
    double      time = 0.0;
    char      OutFileName[100] = { "" };

    double      Freq = 10;      // 생성할 신호의 주파수
    double      nullFreq = 0;   // github push test

    int         idx = 0;
    int         count = 0;

    // 신호 파라미터
    unsigned    i = 0;
    double      Standard = 1;   // DC offset
    double      Amp = 0.5;      // magnitude
    // 신호 저장용 array
    double      OutData[N_STEP] = { 0.0, }; // 측정된 입력(Vin)
    double      OutTime[N_STEP] = { 0.0, }; // 시간
    double      OutVcmd[N_STEP] = { 0.0, }; // 출력신호 (Vcmd)
    double      OutAO1[N_STEP] = { 0.0, };  // ao1 신호 --> 샘플링 후 discrete signal 

    double      Vcmd = 0.0;     // 출력할 전압
    float64      Vin = 0.0;     // 읽어온 전압 (DAQ input)

    // DAQ task 핸들
    TaskHandle   taskAI = 0;    // analog input
    TaskHandle   taskAO = 0;    // analog output
    // ao1
    TaskHandle  taskAO1 = 0;     // ao1 출력용 task
    double      Vsampled = 0.0;  // ai0에서 읽은 샘플값
    double      Vout1 = 0.0;     // ao1로 내보낼 최종 출력값


   // ------------------------------------------------------------
   // DAQ 초기 설정
   // ------------------------------------------------------------
    DAQmxCreateTask("", &taskAI);
    DAQmxCreateTask("", &taskAO);

    // aoi 아날로그 입력 채널 설정
    DAQmxCreateAIVoltageChan(taskAI, "Dev2/ai0", "", DAQmx_Val_RSE, -10.0, 10.0, DAQmx_Val_Volts, "");   // 여기로 읽음
    // ao0 아날로그 출력 채널 설정
    DAQmxCreateAOVoltageChan(taskAO, "Dev2/ao0", "", 0.0, 5.0, DAQmx_Val_Volts, "");                     // 여기로 출력 

    // ao1
    DAQmxCreateTask("", &taskAO1);
    DAQmxCreateAOVoltageChan(taskAO1, "Dev2/ao1", "", 0.0, 5.0, DAQmx_Val_Volts, ""); // taskAI, taskAO 만들고 채널 설정하는 초기화 부분
    
    // DAQ 시작해라
    DAQmxStartTask(taskAI);
    DAQmxStartTask(taskAO);
    DAQmxStartTask(taskAO1);

    double time_test = GetWindowTime();

    printf("Press any key to start the program.... \n");
    getchar();

    // 시작시간 기록
    time_init = GetWindowTime();
    time_curr = time_init;

    // ------------------------------------------------------------
    // 메인 do-while 루프 (샘플링 루프)
    // ------------------------------------------------------------
    do
    {
        time = (time_curr - time_init) * 0.001; // 현재시간 계산 [s]

        /* DAQ Writing : Analog Output */
        Vcmd = Standard + Amp * cos(2.0 * UNIT_PI * Freq * time);   // ----------------------> input signal
        DAQmxWriteAnalogScalarF64(taskAO, 1.0, 5.0, Vcmd, NULL);  // Vcmd를 DAQ로 출력 (analog output) 
                                                                  //'Scalar'가 들어가있는 함수는 단일 인풋/아웃풋임

        /* DAQ Reading */
        DAQmxReadAnalogScalarF64(taskAI, 10.0, &Vin, NULL);     // 입력신호 읽기 (analog input)

        /* ai0로 들어온 합성 신호를 샘플링해서 저장 ===================================*/
        Vsampled = Vin;

        /* 2.5V offset 추가해서 ao1로 출력 */
        Vout1 = Vsampled + 2.5;

        /* 출력 범위 제한 */
        if (Vout1 > 5.0) Vout1 = 5.0;
        if (Vout1 < 0.0) Vout1 = 0.0;

        /* ao1로 출력 =======================================================*/
        DAQmxWriteAnalogScalarF64(taskAO1, 1.0, 5.0, Vout1, NULL);

        /* Memory Write, 데이터 저장 */
        OutTime[count] = time;  // 시간
        OutVcmd[count] = Vcmd;  // ao0
        OutData[count] = Vin;   // x[k] - ai0 ----> 이 순서대로 메모장에 column으로 저장됨
        OutAO1[count] = Vout1;  // y[k] - ao1

        /* check the simulation time and loop count */
        // --------------------------------------------------------
        // 4. 정확한 샘플링 시간 맞추기 (200 Hz 유지)
        // busy-wait 방식
        // --------------------------------------------------------
        while (1)
        {
            time_curr = GetWindowTime();

            if (time_curr - time_init - count * SAMPLING_TIME * 1000 >= (SAMPLING_TIME * 1000.0)) break;
        }
    } while (count++ < N_STEP - 1);

   // ------------------------------------------------------------
   // DAQ 종료
   // ------------------------------------------------------------
    DAQmxStopTask(taskAI);
    DAQmxStopTask(taskAO);

    if (taskAO1) {
        DAQmxStopTask(taskAO1);
        DAQmxClearTask(taskAO1);
    }

    
    // ------------------------------------------------------------
    // 파일 저장  /* Data Print */
    // ------------------------------------------------------------
    sprintf(OutFileName, "%1.1f", SAMPLING_FREQ);   // 파일명 (= 샘플링 주파수)

    pFile = fopen(strcat(OutFileName, "_data.out"), "w+t");

    for (idx = 0; idx < N_STEP; idx++)
    {
        fprintf(pFile, "%20.10f %20.10f %20.10f %20.10f\n", OutTime[idx], OutVcmd[idx], OutData[idx], OutAO1[idx]);
    }

    fclose(pFile);
}