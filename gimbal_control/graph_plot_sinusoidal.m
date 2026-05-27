clear; close all; clc;

%% 데이터 로드
D = readmatrix('sine_A500deg_F0.0250.out', ...
    'FileType', 'text', 'NumHeaderLines', 3);

t          = D(:,1);
Omega_cmd  = D(:,2);   % [deg/s]
Omega      = D(:,6) * 180/pi;  % [rad/s] → [deg/s]

%% 주기 평균
Fs    = 200;
T     = 40;
N_per = T * Fs;                    % 8000 samples/cycle
N_cyc = floor(length(t) / N_per); % 5 cycles

n_use = N_per * N_cyc;

Omega_cmd_m = reshape(Omega_cmd(1:n_use), N_per, N_cyc);
Omega_m     = reshape(Omega(1:n_use),     N_per, N_cyc);

Omega_cmd_avg = mean(Omega_cmd_m, 2);
Omega_avg     = mean(Omega_m,     2);

t_one = (0 : N_per-1).' / Fs;   % 0 ~ 39.995 s

%% 플롯
figure('Position', [100 100 900 650]);

subplot(2,1,1);
plot(t_one, Omega_cmd_avg, 'b', 'LineWidth', 1.5);
ylabel('\Omega_{cmd} [deg/s]');
title(sprintf('Cycle-Averaged Comparison  (A=500 deg/s, f=0.025 Hz, %d cycles)', N_cyc));
grid on;

subplot(2,1,2);
hold on;
plot(t_one, Omega_cmd_avg, 'b--', 'LineWidth', 1.3, 'DisplayName', '\Omega_{cmd} (avg)');
plot(t_one, Omega_avg,     'r',   'LineWidth', 1.5, 'DisplayName', '\Omega measured (avg)');
ylabel('\Omega [deg/s]');
xlabel('Time [s]');
legend('Location', 'best');
grid on;