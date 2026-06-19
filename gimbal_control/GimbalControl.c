#include "NIDAQmx.h"
#include "_macro.h"



// --------------------------------------------------------------- main ----------------------------------------------------------------
void main(void)
{
    int mode = 0;

    printf("Press [Enter] to start the program.... \n\n");
    getchar();

    printf("\n[DAQ Initializing...]\n");
    DAQ_Init();
    // ------ Initialize: switch OFF, motor neutral -------------------
    motor_power(ON, NEUTRAL);

    printf("[Step 0] Confirm motor is stopped, then press [Enter].\n");
    getchar();

    // ------ 0. Calculate Gyro Bias ----------------------------------
    Vg_offset = CalculateGyroBias(N_BIAS);

    do {
        printf("============================================================\n");
        printf("  Hello, are you ready?\n");
        printf("============================================================\n");
        printf("  1. Voltage Sweep\n");
        printf("  2. Frequency Sweep\n");
        printf("  3. Sine Wave Verify\n");
        printf("  4. Triangle Wave Verify\n");
        printf("  5. Static Linearization Verify\n");
        printf("  6. Step Response\n");
        printf("  7. potentiometer positioning\n");
        printf("  8. potentiometer Data Record\n");
        printf("  9. Designation Loop - PD control\n");
        printf("  10. Stabilization Loop - PI control\n");
        printf("  0. Exit\n");
        printf("============================================================\n");
        printf("Select mode > ");

        scanf("%d", &mode);
        while (getchar() != '\n');

        switch (mode)
        {
        case EXIT:
            break;

        case VOLTAGE_SWEEP:
            RunSweep();
            break;
        case FREQ_SWEEP:
            RunBode();
            break;

        case SINE_VALIDATION:
            RunWaveVerify(MODE_SINE);
            break;

        case TRI_VALIDATION:
            RunWaveVerify(MODE_TRI);
            break;

        case STATIC_VALIDATION:
            RunStaticVerify();
            break;

        case STEP_RESPONSE:
            RunStepResponse();
            break;

        case POT_POSITIONING:
            pot_positioning();
            break;

        case POT_DATARECORD:
            RecordPotData();
            break;

        case DESIGNATION:
            RunDesignation();
            break;

        case STABILIZATION:
            RunStabilization();
            break;

        default:
            printf("[Error] Invalid mode: %d\n", mode);
            break;
        }
    } while (mode >= 1 && mode <= 10);


    printf("[DAQ Cleaning up...]\n");
    DAQ_Cleanup();
    printf("\nProgram finished. Press [Enter] to exit.\n");
    getchar();
}
// --------------------------------------------------------------- end main ----------------------------------------------------------------


/* Returns current time [ms] */
double GetWindowTime(void)
{
    LARGE_INTEGER liCounter, liFrequency;
    QueryPerformanceCounter(&liCounter);
    QueryPerformanceFrequency(&liFrequency);
    return (liCounter.QuadPart / (double)(liFrequency.QuadPart) * 1000.0);
}

// for file names!! os timestamp  ---> DO NOT USE IT INSIDE DO-WHILE LOOP
void GetTimestampString(char* buf, size_t bufSize)
{
    time_t now = time(NULL);
    struct tm* t = localtime(&now);
    strftime(buf, bufSize, "%Y%m%d_%H%M%S", t);
}

void memorySet(void)
{
    memset(buftime, 0, sizeof(buftime));
    memset(bufVcmd, 0, sizeof(bufVcmd));
    memset(bufVg, 0, sizeof(bufVg));
    memset(bufVc, 0, sizeof(bufVc));
    memset(bufVpot, 0, sizeof(bufVpot));
    memset(bufomega, 0, sizeof(bufomega));
    memset(bufomega_target, 0, sizeof(bufomega_target));
    memset(buf_disturbance, 0, sizeof(buf_disturbance));
}


void BusyWait_ms(double ms)
{
    double time_start = 0.0;
    time_start = GetWindowTime();
    while (GetWindowTime() - time_start < ms);
}


void WaitNextSample(void)
{
    double target_ms = 0.0;
    target_ms = count * SAMPLING_TIME * 1000.0;
    while (GetWindowTime() - time_init < target_ms); // [ms]
}

/* Check spacebar emergency stop */
int IsEmergencyStop(void)
{
    return (GetAsyncKeyState(VK_SPACE) & 0x8000) ? 1 : 0;
}


double CalculateGyroBias(int nSamples)
{
    double y_bar = 0.0;
    double y_k = 0.0;
    memset(readArr, 0, sizeof(readArr));

    printf("[Gyro Bias] Collecting %d samples...\n", nSamples);
    for (int k = 1; k <= nSamples; k++) {
        int32 err = READ_DATA(readArr);
        y_k = readArr[0];
        if (err != 0) {
            char errBuff[2048];
            DAQmxGetExtendedErrorInfo(errBuff, 2048);
            printf("\n[DAQ Read Error during Bias] %s\n", errBuff);
            return 0.0;
        }

        y_bar = (1.0 - 1.0 / k) * y_bar + (1.0 / k) * y_k;
        BusyWait_ms(5.0);
    }
    printf("[Gyro Bias Done] Vg_offset = %.6f V\n\n", y_bar);
    return y_bar;
}

void motor_power(float64 onoff, double voltage) {

    writeArr[0] = onoff;
    writeArr[1] = (float64)voltage;
    DAQmxWriteAnalogF64(g_taskAO, 1, 1, 10.0, DAQmx_Val_GroupByChannel, writeArr, &sampsPerChanWritten, NULL);
}

void DAQ_Init(void)
{
    DAQmxResetDevice(DAQ_DEV);

    DAQmxCreateTask("", &g_taskAI);
    DAQmxCreateTask("", &g_taskAO);

    DAQmxCreateAIVoltageChan(g_taskAI, DAQ_DEV "/ai2, " DAQ_DEV "/ai3," DAQ_DEV "/ai0", "", DAQmx_Val_RSE, -10.0, 10.0, DAQmx_Val_Volts, "");
    DAQmxCreateAOVoltageChan(g_taskAO, DAQ_DEV "/ao0, " DAQ_DEV "/ao1", "", 0.0, 5.0, DAQmx_Val_Volts, "");

    DAQmxStartTask(g_taskAI);
    DAQmxStartTask(g_taskAO);
}

void DAQ_Cleanup(void)
{
    motor_power(OFF, NEUTRAL);

    DAQmxStopTask(g_taskAI);  DAQmxClearTask(g_taskAI);
    DAQmxStopTask(g_taskAO);  DAQmxClearTask(g_taskAO);

}

void memorySet_bode(void) {
    memset(bode_time, 0, sizeof(bode_time));
    memset(bode_Vcmd, 0, sizeof(bode_Vcmd));
    memset(bode_Vg, 0, sizeof(bode_Vg));
    memset(bode_Vc, 0, sizeof(bode_Vc));
    memset(bode_Vpot, 0, sizeof(bode_Vpot));
    memset(bode_omega, 0, sizeof(bode_omega));
    memset(bode_omega_target, 0, sizeof(bode_omega_target));
}

/* Read 1 sample from DAQ */
void DAQ_ReadSample(void)
{
    memset(readArr, 0, sizeof(readArr));
    Vg = 0, Vpot = 0, omega = 0;
    int32 err = 0;
    err = READ_DATA(readArr);
    if (err != 0) {
        char errBuff[2048];
        DAQmxGetExtendedErrorInfo(errBuff, 2048);
        printf("[DAQ Read Error] %s\n", errBuff);
    }
    Vg = readArr[0];    // ai2
    Vpot = readArr[1];  // ai3
    omega = K_GIMBAL * (Vg - Vg_offset);
    disturbance = readArr[2];
}


int BuildVoltageSequence(void)
{
    // initialize
    double deltas[N_STEPS_MAX] = { 0.0 };
    int    nDeltas = 0;
    double vCW = 0.0;
    double vCCW = 0.0;
    int nSteps = 0;

    for (int i = 1; i <= 50; i++)  deltas[nDeltas++] = i * 0.01;        /* deadzone */
    for (int i = 1; i <= 39; i++)  deltas[nDeltas++] = 0.50 + i * 0.05; /* outer */
    deltas[nDeltas++] = 2.50;

    for (int i = 0; i < nDeltas; i++) {
        vCW = 2.5 + deltas[i]; if (vCW > 5.0) vCW = 5.0;
        vCCW = 2.5 - deltas[i]; if (vCCW < 0.0) vCCW = 0.0;
        voltSeq[nSteps++] = vCW;   /* CW  (even index) */
        voltSeq[nSteps++] = vCCW;  /* CCW (odd index)  */
    }
    return nSteps;
}


void RunSweep(void)
{
    // initialize
    double omega_sum = 0;
    double omega_i = 0;
    const char* dir;
    char filename[256];
    int nStep = 0;

    nStep = BuildVoltageSequence();

    char* outputDir = "motor_sweep_data";
    _mkdir(outputDir);

    printf("============================================================\n");
    printf("  [MODE 1] Voltage Sweep (%d steps)\n", N_STEPS_MAX);
    printf("  Deadzone 2.0~3.0V -> 0.01V step\n");
    printf("  Outer    0.0~2.0V, 3.0~5.0V -> 0.05V step\n");
    printf("  Hold time : %.1f sec,  K_gimbal = %.4f (rad/s)/V\n", HOLD_TIME, K_GIMBAL);
    printf("============================================================\n\n");

    printf("[Step 1] Turn on gimbal switch, then press [Enter].\n\n");
    getchar();
    GetAsyncKeyState(VK_SPACE);

    // apply voltage, Vcmd
    for (int step = 0; step < nStep && !IsEmergencyStop(); step++)
    {
        dir = (step % 2 == 0) ? "CW" : "CCW";

        memorySet();
        Vcmd = voltSeq[step];
        time_init = GetWindowTime();
        count = 0;

        printf("-----------------------------------------\n");
        printf("[Step %3d/%d]  Vcmd = %.2f V  (%s)\n", step + 1, nStep, Vcmd, dir);

        /* Apply voltage */
        motor_power(ON, Vcmd);

        // ------------------------ main loop -----------------------------------
        do {
            DAQ_ReadSample();
            time_elapsed = (GetWindowTime() - time_init) * 0.001;

            if (count < N_HOLD) {
                buftime[count] = time_elapsed;
                bufVcmd[count] = Vcmd;
                bufVg[count] = Vg;
                bufVpot[count] = Vpot;
                bufomega[count] = omega;
            }

            count++;
            WaitNextSample();
        } while (!IsEmergencyStop() && (time_elapsed < HOLD_TIME));
        // ------------------------ end while -----------------------------------

        /* Neutral pause between steps */
        if (!IsEmergencyStop() && step < N_STEPS_MAX - 1) {
            motor_power(ON, NEUTRAL);
            BusyWait_ms(1000.0);
        }

        /* Save to file */
        if (count > 0) {
            sprintf(filename, "%s/step_%03d_V%.2f_%s.out", outputDir, step + 1, Vcmd, dir);
            FILE* pFile = fopen(filename, "w");
            if (pFile) {
                fprintf(pFile, "%% Sweep Step %d/%d  Vcmd=%.4fV  %s \n", step + 1, N_STEPS_MAX, Vcmd, dir);
                fprintf(pFile, "%% Vg_offset=%.6fV  K_gimbal=%.6f\n\n", Vg_offset, K_GIMBAL);
                fprintf(pFile, "Time[s]              Vcmd[V]              Vg_raw[V]            Pot[V]             Omega[rad/s]\n");

                savecount = (count < N_HOLD) ? count : N_HOLD;
                for (int i = 0; i < savecount; i++)
                    fprintf(pFile, "%20.10f %20.10f %20.10f %20.10f %20.10f\n",
                        buftime[i], bufVcmd[i], bufVg[i], bufVpot[i], bufomega[i]);
                fclose(pFile);
                printf("  -> Saved: %s  (%d samples)\n", filename, count);
            }
            else
            {
                printf("  !! File open failed: %s\n", filename);
            }
        }

    }

    motor_power(ON, NEUTRAL);
    printf("\n[MODE 1 Done] Output folder: %s\n\n", outputDir);
}


double Triangle_cmd(double t)
{
    double phase = 0;
    phase = fmod(t, TRI_PERIOD) / TRI_PERIOD;
    if (phase < 0.25) return  TRI_AMP_DEGS * (4.0 * phase);
    else if (phase < 0.75) return  TRI_AMP_DEGS * (2.0 - 4.0 * phase);
    else                   return  TRI_AMP_DEGS * (4.0 * phase - 4.0);
}


double InverseMap(double omega_c_deg)
{
    double w = 0.0;
    double Vc_pos_DZ = 0.0;
    double Vc_neg_DZ = 0.0;
    double alpha = 0.0;

    // clamping at saturation
    if (omega_c_deg > WC_SAT) omega_c_deg = WC_SAT;
    if (omega_c_deg < -WC_SAT) omega_c_deg = -WC_SAT;

    //  !!! omega_target must be [rad/s] since controller gains are all [rad/s] !!!
    omega_target = omega_c_deg * DEG2RAD;   // [rad/s]

    // calculating deadzone boundary
    w = WC_DZ;
    Vc_pos_DZ = CW_COEFF4 * pow(w, 4) + CW_COEFF3 * pow(w, 3) + CW_COEFF2 * pow(w, 2) + CW_COEFF1 * w + CW_COEFF0;
    w = -WC_DZ;
    Vc_neg_DZ = CCW_COEFF4 * pow(w, 4) + CCW_COEFF3 * pow(w, 3) + CCW_COEFF2 * pow(w, 2) + CCW_COEFF1 * w + CCW_COEFF0;

    Vc = NEUTRAL;

    if (omega_c_deg >= WC_DZ)  // cw
    {
        Vc = CW_COEFF4 * pow(omega_c_deg, 4) + CW_COEFF3 * pow(omega_c_deg, 3) + CW_COEFF2 * pow(omega_c_deg, 2) + CW_COEFF1 * omega_c_deg + CW_COEFF0;
    }
    else if (omega_c_deg <= -WC_DZ)   // ccw
    {
        Vc = CCW_COEFF4 * pow(omega_c_deg, 4) + CCW_COEFF3 * pow(omega_c_deg, 3) + CCW_COEFF2 * pow(omega_c_deg, 2) + CCW_COEFF1 * omega_c_deg + CCW_COEFF0;
    }
    else  // deadzone - linear curvefit
    {
        // alpha=0 → -WC_DZ (Vc_neg_DZ),  alpha=1 → +WC_DZ (Vc_pos_DZ)
        alpha = (omega_c_deg - (-WC_DZ)) / (2.0 * WC_DZ);
        Vc = Vc_neg_DZ + alpha * (Vc_pos_DZ - Vc_neg_DZ);
    }

    // output clamping
    if (Vc < 0.0) Vc = 0.0;
    if (Vc > 5.0) Vc = 5.0;
    return Vc;
}

void RunWaveVerify(int mode)
{
    double T_total = 0.0;
    char filename[256];
    char timestamp[32];
    double omega_c = 0.0;   /* 파형 명령값 [deg/s] */
    const char* outputDir = "RunWaveVerify";
    _mkdir(outputDir);

    memorySet();

    printf("Turn on gimbal switch, then press [Enter].\n");
    getchar();
    GetAsyncKeyState(VK_SPACE);
    motor_power(ON, NEUTRAL);

    T_total = (mode == MODE_SINE) ? SINE_T_TOTAL : TRI_T_TOTAL;
    time_init = GetWindowTime();
    time_elapsed = 0.0;
    count = 0;

    // ---------------------------------------- main loop ---------------------------------------------------
    do {

        time_elapsed = (GetWindowTime() - time_init) * 0.001;

        omega_c = (mode == MODE_SINE) ? SINE_CMD_DEGS(time_elapsed) : Triangle_cmd(time_elapsed);   /* omega_c: [deg/s]  */
        Vc = InverseMap(omega_c);   /* omega_c[deg/s] → Vc[V] */

        // apply voltage Vc
        motor_power(ON, Vc);
        DAQ_ReadSample();

        if (count < BUF_SIZE) {
            buftime[count] = time_elapsed;
            buf_OmegaCmd[count] = omega_c;  // [deg/s]
            bufVc[count] = Vc;
            bufVg[count] = Vg;
            bufVpot[count] = Vpot;
            bufomega[count] = omega;
            bufomega_target[count] = omega_target;
        }
        count++;

        //if (count % 2000 == 0) // print once every 10s
        //    printf("  [%5.1f / %5.1f s]  omega_c=%7.2f deg/s  Vc=%7.4f  omega=%8.4f rad/s\n",
        //        time_elapsed, T_total, omega_c, Vc, omega);

        WaitNextSample();
    } while (!IsEmergencyStop() && (time_elapsed < T_total));
    // ---------------------------------------- main loop ---------------------------------------------------

    motor_power(ON, NEUTRAL);
    savecount = (count < BUF_SIZE) ? count : BUF_SIZE;

    // save files
    GetTimestampString(timestamp, sizeof(timestamp));

    if (mode == MODE_SINE) {
        sprintf(filename, "%s/sine_A%.0fdeg_F%.4f_%s.out", outputDir, SINE_AMP_DEGS, SINE_FREQ, timestamp);
    }
    else
        sprintf(filename, "%s/tri_A%.0fdeg_F%.4f_%s.out", outputDir, TRI_AMP_DEGS, 1.0 / TRI_PERIOD, timestamp);

    FILE* fp = fopen(filename, "w");
    if (!fp) { printf("[Error] Cannot open: %s\n", filename); return; }

    fprintf(fp, "Time[s]              OmegaCmd[deg/s]      Vc[V]"
        "                Vg[V]                Pot[V]             Omega_out[deg/s]           Omega_target[deg/s]\n");
    for (int i = 0; i < count; i++)
        fprintf(fp, "%20.10f %20.10f %20.10f %20.10f %20.10f %20.10f %20.10f\n",
            buftime[i], buf_OmegaCmd[i], bufVc[i], bufVg[i], bufVpot[i], bufomega[i]*RAD2DEG, bufomega_target[i]*RAD2DEG);

    fclose(fp);
    printf("[Saved] %s  (%d samples)\n", filename, count);
}


void RunBode(void)
{
    double t = 0.0;
    double freq = 0.0;
    double T_total = 0.0;
    double total_est = 0.0;
    int    N_total = 0;
    char   fname[256];
    int j = 0;
    double freq_ = 0.1;
    double omega_c = 0.0;   /* Bode 사인 명령값 [deg/s] */

    do {
        freq_step[j] = freq_;
        j++;
        freq_ = freq_ + 0.1;
    } while (freq_ <= 5.0);

    const char* outputDir = "bode_data";
    _mkdir(outputDir);
    for (int i = 0; i < N_FREQS; i++)
        total_est += (1.0 / freq_step[i]) * N_CYCLES;
    printf("============================================================\n");
    printf("  [MODE 4] Frequency Response (Bode Plot)\n");
    printf("  Sine amplitude : %.0f deg/s  (omega_c)\n", BODE_SINE_AMP_DEGS);
    printf("  Frequencies    : %d points  (0.1 ~ 3.0 Hz)\n", N_FREQS);
    printf("  Estimated time : ~%.0f sec (%.1f min)\n", total_est, total_est / 60.0);
    printf("============================================================\n\n");
    printf("[Step 1] Turn on gimbal switch, then press [Enter].\n\n");

    getchar();
    GetAsyncKeyState(VK_SPACE);

    motor_power(ON, NEUTRAL);

    for (int fi = 0; fi < N_FREQS && !IsEmergencyStop(); fi++)
    {
        freq = freq_step[fi];
        T_total = (1.0 / freq) * N_CYCLES;
        N_total = (int)(T_total * SAMPLING_FREQ);
        memorySet_bode();

        if (N_total > BODE_N_MAX) {
            printf("[WARN] f=%.1f Hz skipped (N_total=%d exceeds BODE_N_MAX=%d)\n",
                freq, N_total, BODE_N_MAX);
            continue;
        }

        printf("------------------------------------------------------------\n");
        printf("[%d/%d] f=%.1fHz | cycles=%d | T=%.1fs | N_total=%d\n",
            fi + 1, N_FREQS, freq, N_CYCLES, T_total, N_total);

        time_init = GetWindowTime();
        count = 0;

        // ----------------------------------------- do-while loop ----------------------------------------------
        do
        {
            DAQ_ReadSample();
            t = count * SAMPLING_TIME;

            /* 명령 단위: [deg/s]  (기존 Vcmd[V] → omega_c[deg/s] 로 변경) */
            omega_c = BODE_SINE_AMP_DEGS * sin(2.0 * UNIT_PI * freq * t);
            Vc = InverseMap(omega_c);   /* omega_c[deg/s] → Vc[V] */
            motor_power(ON, Vc);

            bode_time[count] = t;
            bode_Vcmd[count] = omega_c;     /* [deg/s] 저장 */
            bode_Vc[count] = Vc;
            bode_Vg[count] = Vg;
            bode_Vpot[count] = Vpot;
            bode_omega[count] = omega;
            bode_omega_target[count] = omega_target;

            count++;
            WaitNextSample();

        } while (count < N_total && !IsEmergencyStop());
        // ----------------------------------------- end do-while loop ----------------------------------------------

        if (IsEmergencyStop()) {
            printf("\n[EMERGENCY STOP] Spacebar pressed!\n");
            break;
        }

        motor_power(ON, NEUTRAL);

        sprintf(fname, "%s/raw_f%.2fHz.out", outputDir, freq);
        FILE* fp = fopen(fname, "w+t");
        if (fp) {
            fprintf(fp, "%% f=%.2fHz  n_cycles=%d  N_total=%d  dt=%.6f\n",
                freq, N_CYCLES, N_total, SAMPLING_TIME);
            fprintf(fp, "%% Amp=%.2fdeg/s\n", BODE_SINE_AMP_DEGS);
            fprintf(fp, "%-20s %-20s %-20s %-20s %-20s %-20s %-20s\n",
                "Time[s]", "OmegaCmd[deg/s]", "Vc[V]", "Vg[V]", "Pot[V]",
                "Omega[deg/s]", "Omega_target[deg/s]");
            for (int k = 0; k < count; k++)   // ★ N_total → count (비상정지 대응)
                fprintf(fp, "%20.10f %20.10f %20.10f %20.10f %20.10f %20.10f %20.10f\n",
                    bode_time[k], bode_Vcmd[k], bode_Vc[k], bode_Vg[k], bode_Vpot[k],
                    bode_omega[k]*RAD2DEG, bode_omega_target[k]*RAD2DEG);
            fclose(fp);
            printf("  -> Saved: %s  (%d samples)\n", fname, count);
        }
        else {
            printf("  !! File open failed: %s\n", fname);
        }

        BusyWait_ms(1000.0);
    }

    motor_power(ON, NEUTRAL);
    printf("\n[MODE 4 Done] Output folder: %s\n\n", outputDir);
}



void RunStaticVerify(void)
{
    double static_cmd[STATIC_N_STEPS] = { 0.0 };   // [deg/s] 
    int    n_static = 0;
    double omega_c_step = 0.0;   // 현재 스텝 명령값 [deg/s] 
    int avg_start = 0;
    int avg_n = 0;
    double omega_sum = 0.0;
    double Vc_sum = 0.0;
    double omega_avg = 0.0;
    double Vc_avg = 0.0;

    // 명령 배열 생성: [deg/s] (STATIC_CMD_STEP [deg/s] 간격) 
    for (double v = STATIC_CMD_STEP; v <= STATIC_CMD_MAX_DEGS + 1e-9; v += STATIC_CMD_STEP) {
        static_cmd[n_static++] = v;   // CW  [deg/s] 
        static_cmd[n_static++] = -v;   // CCW [deg/s]
    }   /* n_static == 51 (STATIC_CMD_MAX_DEGS/STATIC_CMD_STEP * 2 + 1 이하) */

    const char* outputDir = "static_verify_data";
    _mkdir(outputDir);

    char sumname[256];
    sprintf(sumname, "%s/summary.out", outputDir);
    FILE* fpSum = fopen(sumname, "w+t");
    if (fpSum) {
        fprintf(fpSum, "%% Static Linearization Verify\n");
        fprintf(fpSum, "%% Vg_offset=%.6f  K_gimbal=%.6f\n",
            Vg_offset, K_GIMBAL);
        fprintf(fpSum, "%-5s %-20s %-20s %-20s %-20s\n",
            "Step", "OmegaCmd[deg/s]", "Vc[V]",
            "Omega_avg[rad/s]", "Omega_target[rad/s]");
    }
    else {
        printf("  !! Summary file open failed: %s\n", sumname);
    }

    printf("============================================================\n");
    printf("  [MODE 5] Static Linearization Verify (%d steps)\n", n_static);
    printf("  OmegaCmd : +%.0f, -%.0f, ..., +%.0f, -%.0f  [deg/s]\n",
        STATIC_CMD_STEP, STATIC_CMD_STEP, STATIC_CMD_MAX_DEGS, STATIC_CMD_MAX_DEGS);
    printf("  Hold : %.1f s  |  Avg last : %.1f s (%d samples)\n",
        HOLD_TIME, STATIC_AVG_TIME, STATIC_AVG_N);
    printf("============================================================\n\n");

    printf("[Step 1] Turn on gimbal switch, then press [Enter].\n\n");
    getchar();
    GetAsyncKeyState(VK_SPACE);


    // initialize motor
    motor_power(ON, NEUTRAL);

    // step loop
    for (int step = 0; step < n_static && !IsEmergencyStop(); step++)
    {
        omega_sum = 0.0;
        Vc_sum = 0.0;
        memorySet();

        /* 명령값: [deg/s]  (기존 Vcmd[V] → omega_c[deg/s] 로 변경) */
        omega_c_step = static_cmd[step];
        Vc = InverseMap(omega_c_step);   /* omega_c[deg/s] → Vc[V] */

        time_init = GetWindowTime();
        time_elapsed = 0.0;
        count = 0;

        printf("-----------------------------------------\n");
        printf("[Step %3d/%d]  OmegaCmd = %+.2f deg/s  "
            "Vc = %.4f V  omega_target = %+.4f rad/s\n",
            step + 1, n_static, omega_c_step, Vc, omega_target);

        // --------------------------------------------- do-while loop ----------------------------------------
        do {
            DAQ_ReadSample();
            time_elapsed = (GetWindowTime() - time_init) * 0.001;

            motor_power(ON, Vc);

            if (count < N_HOLD) {
                buftime[count] = time_elapsed;
                bufVcmd[count] = omega_c_step;   /* [deg/s] 저장 */
                bufVc[count] = Vc;
                bufVg[count] = Vg;
                bufVpot[count] = Vpot;
                bufomega[count] = omega;
                bufomega_target[count] = omega_target;
            }

            count++;
            WaitNextSample();

        } while (!IsEmergencyStop() && (time_elapsed < HOLD_TIME));
        // --------------------------------------------- end do-while loop ----------------------------------------

        // prevent buffer overflow
        savecount = (count < N_HOLD) ? count : N_HOLD;
        // get the last part of data (steady-state)
        avg_start = savecount - STATIC_AVG_N;
        if (avg_start < 0) avg_start = 0;
        avg_n = savecount - avg_start;

        for (int i = avg_start; i < savecount; i++) {
            omega_sum += bufomega[i];
            Vc_sum += bufVc[i];
        }
        omega_avg = (avg_n > 0) ? omega_sum / avg_n : 0.0;
        Vc_avg = (avg_n > 0) ? Vc_sum / avg_n : Vc;

        printf("  -> omega_avg = %+.4f rad/s  "
            "(last %.1f s, %d samples)\n",
            omega_avg, STATIC_AVG_TIME, avg_n);

        // save raw files
        char filename[256];
        sprintf(filename, "%s/step_%03d_OmegaCmd%+.0fdeg.out",
            outputDir, step + 1, omega_c_step);
        FILE* pFile = fopen(filename, "w+t");
        if (pFile) {
            fprintf(pFile,
                "%% Static Verify Step %d/%d  OmegaCmd=%+.4f deg/s\n",
                step + 1, n_static, omega_c_step);
            fprintf(pFile,
                "%% Vg_offset=%.6fV  K_gimbal=%.6f\n",
                Vg_offset, K_GIMBAL);
            fprintf(pFile,
                "%% omega_avg(last %.1fs)=%.6f rad/s  "
                "omega_target=%.6f rad/s\n\n",
                STATIC_AVG_TIME, omega_avg, omega_target);
            fprintf(pFile,
                "%-20s %-20s %-20s %-20s %-20s %-20s %-20s\n",
                "Time[s]", "OmegaCmd[deg/s]", "Vc[V]",
                "Vg[V]", "Pot[V]",
                "Omega[deg/s]", "Omega_target[deg/s]");
            for (int i = 0; i < savecount; i++)
                fprintf(pFile,
                    "%20.10f %20.10f %20.10f "
                    "%20.10f %20.10f %20.10f %20.10f\n",
                    buftime[i], bufVcmd[i], bufVc[i],
                    bufVg[i], bufVpot[i],
                    bufomega[i] * RAD2DEG, bufomega_target[i]*RAD2DEG);
            fclose(pFile);
            printf("  -> Saved : %s  (%d samples)\n", filename, savecount);
        }
        else {
            printf("  !! File open failed: %s\n", filename);
        }
        if (fpSum)
            fprintf(fpSum, "%-5d %20.10f %20.10f %20.10f %20.10f\n",
                step + 1, omega_c_step, Vc_avg, omega_avg, omega_target);

        // close, turn OFF motor
        if (!IsEmergencyStop() && step < n_static - 1) {
            motor_power(ON, NEUTRAL);
            BusyWait_ms(1000.0);
        }
    }  // end for

    if (IsEmergencyStop())
        printf("\n[EMERGENCY STOP] Spacebar pressed!\n");

    motor_power(ON, NEUTRAL);

    if (fpSum) {
        fclose(fpSum);
        printf("\n[Summary] Saved : %s\n", sumname);
    }
    printf("[MODE 5 Done] Output folder: %s\n\n", outputDir);
}


// step response
void RunStepResponse(void)
{
    char filename[256];
    char timestamp[32];

    const char* outputDir = "step_response_data";
    _mkdir(outputDir);

    printf("============================================================\n");
    printf("  [MODE 6] Step Response\n");
    printf("  Step input  : %.2f deg/s  (STEP_INPUT_DEGS)\n", STEP_INPUT_DEGS);
    printf("  Settle time : %.1f s  (Keep neutral before Step Input...)\n", STEP_SETTLE_TIME);
    printf("  Record time : %.1f s  (Time recorded after Step Input)\n", RECORD_TIME);
    printf("  K_gimbal = %.4f\n", K_GIMBAL);
    printf("============================================================\n\n");

    printf("[Step 1] Turn on gimbal switch, then press [Enter].\n\n");
    getchar();
    GetAsyncKeyState(VK_SPACE);

    // initialize
    memorySet();
    motor_power(ON, NEUTRAL);
    printf("[Settling] %.1f s at NEUTRAL...\n", STEP_SETTLE_TIME);
    BusyWait_ms(STEP_SETTLE_TIME * 1000.0);

    if (IsEmergencyStop()) {
        printf("\n[EMERGENCY STOP] Spacebar pressed!\n");
        motor_power(ON, NEUTRAL);
        return;
    }

    // 명령 단위: [deg/s] 
    double omega_c_step = STEP_INPUT_DEGS;
    Vc = InverseMap(omega_c_step);  

    printf("[Step] Applying OmegaCmd = %+.2f deg/s  ->  Vc = %.4f V  "
        "(omega_target = %+.4f rad/s)\n\n", omega_c_step, Vc, omega_target);

    // apply voltage
    motor_power(ON, Vc);
    time_init = GetWindowTime();
    time_elapsed = 0.0;
    count = 0;

    // ---------------------------------------------- do-while loop --------------------------------------
    do {
        DAQ_ReadSample();
        time_elapsed = (GetWindowTime() - time_init) * 0.001;

        motor_power(ON, Vc);

        if (count < BUF_SIZE) {
            buftime[count] = time_elapsed;
            bufVcmd[count] = omega_c_step;   /* [deg/s] 저장 */
            bufVc[count] = Vc;
            bufVg[count] = Vg;
            bufVpot[count] = Vpot;
            bufomega[count] = omega;
            bufomega_target[count] = omega_target;
        }

       /* if (count % (int)SAMPLING_FREQ == 0)
            printf("  [%5.2f / %.1f s]  omega = %+8.4f rad/s\n",
                time_elapsed, RECORD_TIME, omega);*/

        count++;
        WaitNextSample();

    } while (!IsEmergencyStop() && (time_elapsed < RECORD_TIME));
    // ---------------------------------------------- end do-while loop --------------------------------------

    motor_power(ON, NEUTRAL);

    if (IsEmergencyStop())
        printf("\n[EMERGENCY STOP] Spacebar pressed!\n");

    savecount = (count < BUF_SIZE) ? count : BUF_SIZE;
    GetTimestampString(timestamp, sizeof(timestamp));

    sprintf(filename, "%s/step_OmegaCmd%+.0fdeg_%s.out", outputDir, omega_c_step, timestamp);
    FILE* fp = fopen(filename, "w+t");
    if (fp) {
        fprintf(fp, "%% Step Response  OmegaCmd=%+.2fdeg/s  Vc=%.6fV\n", omega_c_step, Vc);
        fprintf(fp, "%% Vg_offset=%.6fV  K_gimbal=%.6f\n",
            Vg_offset, K_GIMBAL);
        fprintf(fp, "%% omega_target=%.6f rad/s  "
            "settle=%.1fs  record=%.1fs  samples=%d\n\n",
            omega_target, STEP_SETTLE_TIME, RECORD_TIME, savecount);
        fprintf(fp, "%-20s %-20s %-20s %-20s %-20s %-20s %-20s\n",
            "Time[s]", "OmegaCmd[deg/s]", "Vc[V]",
            "Vg[V]", "Pot[V]", "Omega[deg/s]", "Omega_target[deg/s]");
        for (int i = 0; i < savecount; i++)
            fprintf(fp, "%20.10f %20.10f %20.10f %20.10f %20.10f %20.10f %20.10f\n",
                buftime[i], bufVcmd[i], bufVc[i],
                bufVg[i], bufVpot[i], bufomega[i]*RAD2DEG, bufomega_target[i]*RAD2DEG);
        fclose(fp);
        printf("\n[Saved] %s  (%d samples)\n", filename, savecount);
    }
    else {
        printf("  !! File open failed: %s\n", filename);
    }

    printf("[MODE 6 Done] Output folder: %s\n\n", outputDir);
}


void pot_positioning(void)
{

    // initialize
    int keyboard_input = 0;

    count = 0.0;
    time_init = 0.0;

    printf("============================================================\n");
    printf("  [Potentiometer Positioning]\n");
    printf("  <- CW (left)    -> CCW (right)    s: end \n");
    printf("============================================================\n");


    // initialize motor
    motor_power(ON, NEUTRAL);
    time_init = GetWindowTime();

    // main loop
    do {

        if (_kbhit()) { /* ------------------------------------------------------------------------------- */

            keyboard_input = _getch();
            if (keyboard_input == SPECIAL_KEY) {    // recognize special key
                keyboard_input = _getch();

                if (keyboard_input == RIGHT_KEY)
                    Vc = NEUTRAL + EPS;       // move CW
                else if (keyboard_input == LEFT_KEY)
                    Vc = NEUTRAL - EPS;       // move CCW
            }
        }
        else {
            Vc = NEUTRAL;     // stop motor
        }       /* ---------------------------------------------------------------------- decide Vcmd ------------ */

        motor_power(ON, Vc);      // apply Vcmd
        DAQ_ReadSample();
        count++;
        WaitNextSample();   // busy-wait

    } while (!IsEmergencyStop() && keyboard_input != 's');    // end while

    if (IsEmergencyStop())
        printf("\n[EMERGENCY STOP] Spacebar pressed!\n");

    motor_power(ON, NEUTRAL);

    printf("\n------ [Positioning Done] Vpot = %.4f --------\n\n", Vpot);

}


void RecordPotData(void)
{
    char filename[256];
    int file_deg = 0;

    memorySet();

    const char* outputDir = "Pot_Modeling";
    _mkdir(outputDir);

    printf("============================================================\n");
    printf("  [MODE 8] Potentiometer Recording\n");
    printf("  Record time : %.1f s\n", RECORD_TIME);
    printf("  Sampling    : %.0f Hz\n", SAMPLING_FREQ);
    printf("============================================================\n\n");

    printf("[Step 1] Turn on gimbal switch, then press [Enter].\n\n");
    getchar();
    GetAsyncKeyState(VK_SPACE);

    motor_power(ON, NEUTRAL);

    time_init = GetWindowTime();
    time_elapsed = 0.0;
    count = 0;

    printf("[Recording] %.1f s ...\n", RECORD_TIME);

    // ----------------------------------------------- do-while loop ---------------------------------------
    do {
        DAQ_ReadSample();
        time_elapsed = (GetWindowTime() - time_init) * 0.001;

        if (count < BUF_SIZE) {
            buftime[count] = time_elapsed;
            bufVg[count] = Vg;
            bufVpot[count] = Vpot;
            bufomega[count] = omega;
        }

        if (count % (int)SAMPLING_FREQ == 0)
            printf("  [%5.2f / %.1f s]  Vpot = %.4f V\n",
                time_elapsed, RECORD_TIME, Vpot);

        count++;
        WaitNextSample();

    } while (!IsEmergencyStop() && (time_elapsed < RECORD_TIME));
    // ----------------------------------------------- do-while loop ---------------------------------------

    motor_power(ON, NEUTRAL);

    if (IsEmergencyStop())
        printf("\n[EMERGENCY STOP] Spacebar pressed!\n");

    // save files
    savecount = (count < BUF_SIZE) ? count : BUF_SIZE;

    printf("Enter the angle [deg] cw \n");
    scanf("%d", &file_deg);

    sprintf(filename, "%s/pot_record_ccw%d.out", outputDir, file_deg);
    FILE* fp = fopen(filename, "w+t");
    if (fp) {
        fprintf(fp, "%% Potentiometer Recording  %.1f s\n", RECORD_TIME);
        fprintf(fp, "%% Vg_offset=%.6fV  K_gimbal=%.6f\n", Vg_offset, K_GIMBAL);
        fprintf(fp, "%% Fs=%.0f Hz  samples=%d\n\n", SAMPLING_FREQ, savecount);
        fprintf(fp, "%-20s %-20s %-20s %-20s\n",
            "Time[s]", "Vg[V]", "Pot[V]", "Omega[rad/s]");
        for (int i = 0; i < savecount; i++)
            fprintf(fp, "%20.10f %20.10f %20.10f %20.10f\n",
                buftime[i], bufVg[i], bufVpot[i], bufomega[i]);
        fclose(fp);
        printf("\n[Saved] %s  (%d samples)\n", filename, savecount);
    }
    else {
        printf("  !! File open failed: %s\n", filename);
    }
    while (getchar() != '\n') { ; }

    printf("[MODE 8 Done]\n\n");
}


void RunDesignation(void)
{
    char filename[256];
    char timestamp[32];
    int k = 0;
    double psi_cmd_deg = 0.0;      /* absolute input angle        [deg]   */
    double psi_now_deg = 0.0;      /* absolute current angle      [deg]   */
    double eps_deg = 0.0;           /* epsilon                   [deg]   */
    double eps_rad = 0.0;
    double omega_deg = 0.0;        /* omega measured              [deg/s] */
    double omega_U_deg = 0.0;      /* omega_cmd                   [deg/s] */
    double omega_U_rad = 0.0;

    const char* outputDir = "designation_data";
    _mkdir(outputDir);

    printf("============================================================\n");
    printf("  [MODE 9] Designation Loop (PD Position Control)\n");
    printf("  Kp = %.4f [1/s]   Kd = %.4f [-]\n", KP, KD);
    printf("  K_pot = %.6f [deg/V]   (psi = K_pot*(Vpot-NEUTRAL))\n", K_POT);
    printf("  Record time : %.1f s\n", RECORD_TIME);
    printf("============================================================\n\n");

    printf("Enter target angle [deg]  (NEUTRAL, + : CW,  - : CCW) : ");
    scanf("%lf", &psi_cmd_deg);
    while (getchar() != '\n');

    GetAsyncKeyState(VK_SPACE);

    memorySet();
    motor_power(ON, NEUTRAL);
    BusyWait_ms(LOOP_SETTLE_TIME * 1000.0);


    DAQ_ReadSample();
    psi_now_deg = K_POT * (Vpot - PSI_NEUTRAL);
    printf("[Init]   Vpot = %.4f V   psi_now = %+.3f deg\n",
        Vpot, psi_now_deg);
    printf("[Target] psi_cmd = %+.3f deg\n\n", psi_cmd_deg);

    if (IsEmergencyStop()) {
        printf("\n[EMERGENCY STOP] Spacebar pressed!\n");
        motor_power(ON, NEUTRAL);
        return;
    }
    
    time_init = GetWindowTime();
    time_elapsed = 0.0;
    count = 0;

    // ----------------------------------------- do-while loop ---------------------------------------
    do {
        DAQ_ReadSample();
        time_elapsed = (GetWindowTime() - time_init) * 0.001;

        psi_now_deg = K_POT * (Vpot - PSI_NEUTRAL);     // current [deg]
        eps_deg = psi_cmd_deg - psi_now_deg;        // error[deg]
        eps_rad = eps_deg * DEG2RAD;
        omega_U_rad = KP * eps_rad - KD * omega;    // PD : outer position + inner rate damping
        omega_U_deg = omega_U_rad * RAD2DEG;
        Vc = InverseMap(omega_U_deg);       

        motor_power(ON, Vc);        // apply Vc

        if (count < BUF_SIZE) {
            buftime[count] = time_elapsed;
            bufVcmd[count] = omega_U_deg;     /* [deg/s] */
            bufVc[count] = Vc;
            bufVg[count] = Vg;
            bufVpot[count] = Vpot;
            bufomega[count] = omega;            /* [rad/s] */
            bufomega_target[count] = psi_now_deg;      /* [deg] 재활용 */
        }

        /*if (count % (int)SAMPLING_FREQ == 0)
            printf("  [%5.2f s]  psi = %+8.3f / %+8.3f deg   "
                "e = %+8.3f deg   omega_c = %+8.2f deg/s   Vc = %5.3f V\n",
                time_elapsed, psi_now_deg, psi_cmd_deg,
                eps_deg, omega_U_deg, Vc);*/

        count++;
        WaitNextSample();

    } while (!IsEmergencyStop() && (time_elapsed < RECORD_TIME));
    // ----------------------------------------- end do-while loop ---------------------------------------

    motor_power(ON, NEUTRAL);

    if (IsEmergencyStop())
        printf("\n[EMERGENCY STOP] Spacebar pressed!\n");

    // save files
    GetTimestampString(timestamp, sizeof(timestamp));
    savecount = (count < BUF_SIZE) ? count : BUF_SIZE;
    sprintf(filename, "%s/dsg_psi%+.0fdeg_%s.out", outputDir, psi_cmd_deg, timestamp);
    FILE* fp = fopen(filename, "w+t");
    if (fp) {
        fprintf(fp, "%% Designation Loop  psi_cmd=%+.3fdeg  (absolute, NEUTRAL ref)\n",
            psi_cmd_deg);
        fprintf(fp, "%% Kp=%.4f  Kd=%.4f  K_pot=%.6fdeg/V\n",
            KP, KD, K_POT);
        fprintf(fp, "%% Vg_offset=%.6fV  K_gimbal=%.6f  samples=%d\n\n",
            Vg_offset, K_GIMBAL, savecount);
        fprintf(fp, "%-20s %-20s %-20s %-20s %-20s %-20s %-20s\n",
            "Time[s]", "OmegaCmd[deg/s]", "Vc[V]",
            "Vg[V]", "Pot[V]", "Omega[deg/s]", "Psi[deg]");
        for (k = 0; k < savecount; k++)
            fprintf(fp, "%20.10f %20.10f %20.10f %20.10f %20.10f %20.10f %20.10f\n",
                buftime[k], bufVcmd[k], bufVc[k],
                bufVg[k], bufVpot[k], bufomega[k]*RAD2DEG, bufomega_target[k]);
        fclose(fp);
        printf("\n[Saved] %s  (%d samples)\n", filename, savecount);
    }
    else {
        printf("  !! File open failed: %s\n", filename);
    }

    printf("[MODE 9 Done] Output folder: %s\n\n", outputDir);
}



void RunStabilization(void)
{
    char filename[256];
    char timestamp[32];
    int k = 0;

    // controller variables
    double omega_c = 500.0 * DEG2RAD;   // = omega_cmd [rad/s] == 0
    double omegaC = omega_c * RAD2DEG; // for file saving
    double err = 0.0;       //  e = omega_c - omega_h [rad/s]
    double err_prev = 0.0;  // for tustin
    double xI = 0.0;        // integrator [rad]
    double omega_U = 0.0;   // [deg/s] PI-controller output (= motor input)'

    // LPF
    double omega_lpf = 0.0;
    double fc = 20.0;   // lpf bandwidth
    double RC = 1.0 / (2.0 * UNIT_PI * fc);
    double alpha = SAMPLING_TIME / (RC + SAMPLING_TIME);
    double bufomega_raw[BUF_SIZE];

    const char* outputDir = "stabilization_data";
    _mkdir(outputDir);

    printf("============================================================\n");
    printf("  [MODE 10] Stabilization Loop (PI Rate Controller)\n");
    printf("  Kp_stab = %.4f [-]   Ki_stab = %.4f [1/s]\n", KP_STB, KI_STB);
    printf("  omega_c = 0 rad/s  (inertial stabilization)\n");
    printf("  Settle  : %.1f s  |  Record : %.1f s\n", LOOP_SETTLE_TIME, STABILIZATION_TIME);
    printf("============================================================\n\n");

    printf("Turn on gimbal switch, then press [Enter].\n");
    getchar();
    GetAsyncKeyState(VK_SPACE);

    // ------------------------ initialize ----------------------------
    memorySet();
    motor_power(ON, NEUTRAL);
    BusyWait_ms(LOOP_SETTLE_TIME * 1000.0);

    DAQ_ReadSample();
    omega_lpf = omega;
    err_prev = omega_c - omega_lpf;     // omega_c - omega_h

    printf("[start]   Loop Duration = %+.2f [sec]\n\n", STABILIZATION_TIME);

    if (IsEmergencyStop()) {
        printf("\n[EMERGENCY STOP] Spacebar pressed!\n");
        motor_power(ON, NEUTRAL);
        return;
    }


    // ----------------------- main loop -----------------------------------------------------------------
    time_init = GetWindowTime();
    time_elapsed = 0.0;
    count = 0;

    do {
        DAQ_ReadSample();
        time_elapsed = (GetWindowTime() - time_init) * 0.001;

        // ---- LPF 적용 ----
        omega_lpf = omega_lpf + alpha * (omega - omega_lpf);

        // calculate error
        err = omega_c - omega_lpf;      // [rad/s] 

        // tustin method
        xI = xI + (SAMPLING_TIME / 2) * (err + err_prev);   // trapezoidal
        err_prev = err;

        // controller output (= motor input)
        omega_U = KP_STB * err + KI_STB * xI;     // !!! KP*err + KI*\int(err)dt !!!
        omega_U = omega_U * RAD2DEG;              // [deg/s]

        // apply voltage to motor
        Vc = InverseMap(omega_U);
        motor_power(ON, Vc);

        // record data
        if (count < BUF_SIZE) {
            buftime[count] = time_elapsed;
            bufVcmd[count] = omega_U;     // [deg/s]
            bufVc[count] = Vc;
            bufVg[count] = Vg;
            bufVpot[count] = Vpot;
            bufomega[count] = omega;  // signal before lpf
            bufomega_raw[count] = omega_lpf; // signal after lpf
            bufomega_target[count] = err; // [rad/s]
            buf_disturbance[count] = disturbance;
        }

        count++;
        WaitNextSample();

    } while (!IsEmergencyStop() && (time_elapsed < STABILIZATION_TIME));
    // -------------------------------------------------------------- end while ---------------------------------------
    motor_power(ON, NEUTRAL);

    printf("----------- while loop is over ----------------\n\n");
    if (IsEmergencyStop())
        printf("\n[EMERGENCY STOP] Spacebar pressed!\n");

    // save files
    GetTimestampString(timestamp, sizeof(timestamp));
    savecount = (count < BUF_SIZE) ? count : BUF_SIZE;
    sprintf(filename, "%s/stabilization_%.1f[deg_s]_%s.out", outputDir, omegaC, timestamp);
    FILE* fp = fopen(filename, "w+t");
    if (fp) {
        fprintf(fp, "%% Stabilization Loop  omega_c=0 rad/s\n");
        fprintf(fp, "%% Kp_stab=%.4f  Ki_stab=%.4f  (Tustin integrator)\n", KP_STB, KI_STB);
        fprintf(fp, "%% Vg_offset=%.6fV  K_gimbal=%.6f  samples=%d\n\n", Vg_offset, K_GIMBAL, savecount);
        fprintf(fp, "%-20s  %-20s    %-20s   %-20s   %-20s   %-20s   %-20s   %-20s   %-20s\n", 
            "Time[s]", "OmegaU[deg/s]", "Vc[V]", "Vg[V]", "Pot[V]", "Omega_h[deg/s]", "omega_LPF[deg/s]", "Error[deg/s]", "disturbance[V]");
        for (k = 0; k < savecount; k++)
            fprintf(fp, "%20.10f %20.10f %20.10f %20.10f %20.10f %20.10f %20.10f %20.10f %20.10f\n", 
                buftime[k], bufVcmd[k], bufVc[k], bufVg[k], bufVpot[k], bufomega[k]*RAD2DEG, bufomega_raw[k]*RAD2DEG, bufomega_target[k]*RAD2DEG, buf_disturbance[k]);
        fclose(fp);
        printf("\n[Saved] %s  (%d samples)\n", filename, savecount);
    }
    else {
        printf("  !! File open failed: %s\n", filename);
    }

    printf("[MODE 10 Done] Output folder: %s\n\n", outputDir);

}  