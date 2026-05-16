#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <windows.h>
#include <time.h>
#include "NIDAQmx.h"

double GetWindowTime(void)
{
	LARGE_INTEGER	liEndCounter, liFrequency;

	QueryPerformanceCounter(&liEndCounter);
	QueryPerformanceFrequency(&liFrequency);

	return(liEndCounter.QuadPart / (double)(liFrequency.QuadPart) * 1000.0);
};

#define   N_STEP		    (int)   ( FINAL_TIME*SAMPLING_FREQ )
#define   FINAL_TIME		(double)(				5.0 )
#define   SAMPLING_FREQ		(double)(               10 )
#define   SAMPLING_TIME     (double)( 1.0/SAMPLING_FREQ )
#define   UNIT_PI			(double)( 3.14159265358979  )

// ------------------------------------------------------------------------------------------

void main(void)
{
	FILE*		pFile;

	double		time_prev = 0.0;
	double		time_curr = 0.0;
	double		time;
	char		OutFileName[100] = { "" };

	double		Freq = 1.0;

	int			idx = 0;
	int			count = 0;

	unsigned	i = 0;
	double		Standard = 2.5;
	double		OutData[N_STEP] = { 0.0, };
	double		OutTime[N_STEP] = { 0.0, };
	double		OutVcmd[N_STEP] = { 0.0, };

	double		Vcmd = 0.0;
	float64		Vin = 0.0;

	TaskHandle	taskAI = 0;
	TaskHandle	taskAO = 0;

	DAQmxCreateTask("", &taskAI);
	DAQmxCreateTask("", &taskAO);

	DAQmxCreateAIVoltageChan(taskAI, "Dev1/ai0", "", DAQmx_Val_RSE, -10.0, 10.0, DAQmx_Val_Volts, "");
	DAQmxCreateAOVoltageChan(taskAO, "Dev1/ao0", "", 0.0, 5.0, DAQmx_Val_Volts, "");

	DAQmxStartTask(taskAI);
	DAQmxStartTask(taskAO);

	printf("Press any key to start the program.... \n");
	getchar();


	do
	{
		/* check the simulation time and loop count */
		while (1)
		{
			time_curr = GetWindowTime();

			if (time_curr - time_prev >= (SAMPLING_TIME * 1000.0))	break;
		}

		time_prev = time_curr;

		time = SAMPLING_TIME * count;

		/* DAQ Writing : Analog Output */
		Vcmd = Standard * 1.0 * (sin(2.0 * UNIT_PI * Freq * time));
		DAQmxWriteAnalogScalarF64(taskAO, 1.0, 5.0, Vcmd, NULL);

		/* DAQ Reading */
		DAQmxReadAnalogScalarF64(taskAI, 10.0, &Vin, NULL);

		/* Memory Write */
		OutTime[count] = time;
		OutVcmd[count] = Vcmd;
		OutData[count] = Vin;

	} while (count++ < N_STEP - 1);

	DAQmxStopTask(taskAI);
	DAQmxStopTask(taskAO);

	/* Data Print */

	sprintf(OutFileName, "%1.1f", Freq);

	pFile = fopen(strcat(OutFileName, "_data.out"), "w+t");

	for (idx = 0; idx < N_STEP; idx++)
	{
		fprintf(pFile, "%20.10f %20.10f %20.10f\n", OutTime[idx], OutVcmd[idx], OutData[idx]);
	}

	// fcloseall();

}