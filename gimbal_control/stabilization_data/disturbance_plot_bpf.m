% ----------------------------------
%        disturbance pot
% --------------------------------


data = readmatrix("stb_Kp1.0288_Ki29.5203_sine_disturb.out", 'filetype', 'text', ...
    'NumHeaderLines', 5);

time = data(:, 1);
raw_error = data(:, 7);    % [rad/s]
raw_disturb = data(:, 8);  % [V]


Vg_offset = 1.338197; % [V]
K_gimbal = 26.049690; % [(rad/s)/V]
raw_disturb_omega = K_gimbal * (raw_disturb - Vg_offset); % [rad/s]
raw_disturb_omega = raw_disturb_omega * (180/pi); % [deg/s]
raw_error = raw_error * (180/pi); % [deg/s]


% BPF parameter
fs = 200; % sampling freq. [Hz]
fnq = fs/2; % nyquist freq.

Wc_low = 0.3;  
Wc_high = 0.7;
Wc_bpf = [Wc_low, Wc_high] / fnq;

[bpfNum, bpfDen] = butter(1, Wc_bpf, 'bandpass');
[bpfNum_e, bpfDen_e] = butter(1, Wc_bpf, 'bandpass');

offset = 200;

clean_disturb_omega = filtfilt(bpfNum, bpfDen, raw_disturb_omega);
clean_error = filtfilt(bpfNum_e, bpfDen_e, raw_error);


% ---------------------------------------
%   Run Simulink model and get results
% ---------------------------------------
model_name = 'Stabilization_simulink.slx';   % <-- 실제 .slx 파일 이름으로 변경

out = sim(model_name);

t_sim   = out.t_out;        % 시간 벡터
dist_sim   = out.disturbance;       % 시뮬레이션 결과 데이터
omega_h = out.simout;


figure(1);
grid on; hold on;
%plot(time, raw_disturb_omega, 'r--', 'LineWidth', 1);
plot(time, clean_disturb_omega, 'r', 'LineWidth', 1.4);
plot(time, clean_error, 'r--', 'linewidth', 1.4);

plot(t_sim, dist_sim, 'b', 'LineWidth', 1.4);
plot(t_sim, omega_h, 'b--', 'linewidth', 1.4);



xlabel('time [sec]'); ylabel('disturbance [deg/s]');
legend('disturbance[exp]', 'error[exp]', 'disturbance[sim]', 'error[sim]');
title('compare disturbance(\omega_b) w/ error(-\omega_h)');

figure(2);
grid on; hold on;
plot(time, clean_disturb_omega, 'r', 'LineWidth', 1.4);
plot(time, clean_error, 'r--', 'linewidth', 1.4);

xlabel('time [sec]'); ylabel('disturbance [deg/s]');
legend('disturbance[exp]', 'error[exp]');
title('compare disturbance(\omega_b) w/ error(-\omega_h)');

figure(3);
grid on; hold on;

plot(t_sim, dist_sim, 'b', 'LineWidth', 1.4);
plot(t_sim, omega_h, 'b--', 'linewidth', 1.4);


xlabel('time [sec]'); ylabel('disturbance [deg/s]');
legend('disturbance[sim]', 'error[sim]');
title('compare disturbance(\omega_b) w/ error(-\omega_h)');
