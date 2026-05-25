clear; close all; clc;

D = readmatrix('sine_A2.00_F0.0250.out', ...
               'FileType','text', 'NumHeaderLines', 2);
t      = D(:,1);
vcmd   = D(:,2);
om_tgt = D(:,7);
om     = D(:,6);

%% 주기 평균
Fs      = 200;
T       = 40;
N_per   = T * Fs;          % 한 주기당 샘플 수 = 8000
N_cyc   = floor(length(t) / N_per);   % 실제 완성된 주기 수

% 완성된 주기만 사용
n_use   = N_per * N_cyc;
vcmd_m  = reshape(vcmd(1:n_use),   N_per, N_cyc);
om_tgt_m= reshape(om_tgt(1:n_use), N_per, N_cyc);
om_m    = reshape(om(1:n_use),     N_per, N_cyc);

vcmd_avg   = mean(vcmd_m,   2);
om_tgt_avg = mean(om_tgt_m, 2);
om_avg     = mean(om_m,     2);

t_one = (0 : N_per-1).' / Fs;   % 0 ~ 39.995 sec

% ------------ simulation ----------------
num = [790];
den = [1 16.45 172.5];

% input
A = 2.0; % magnitude [V]
freq = 0.025; % [Hz]

dt = 0.005; % sampling period
Tf = 40 ;

t = 0:dt:Tf ;
u = A*sin(2*pi*freq*t) ;

sys = tf(num, den);

y = lsim(sys, u, t);



%% 플롯
figure;
subplot(2,1,1);
plot(t_one, vcmd_avg, 'b', 'LineWidth', 1.2);
ylabel('Vcmd_{ref} [V]');
title('주기 평균 (Vcmd=1.5[V], f=0.025[Hz])');
grid on;

% subplot(2,1,2); 
figure(2);
hold on;
plot(t_one, om_tgt_avg*180/pi, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Target \omega (avg)');
plot(t_one, om_avg*180/pi,     'r',   'LineWidth', 1.2, 'DisplayName', 'Measured \omega (avg)');
plot(t, y*180/pi, 'g', 'LineWidth', 1.5, 'DisplayName', 'Simulation \omega (avg)');
title('주기 평균 (Vcmd=1.5[V], f=0.025[Hz])');
ylabel('\omega [deg/s]'); xlabel('Time [s]');
legend; grid on;

y_trim = y(1:N_per);
err_abs_mean = mean(abs(om_avg - y_trim(:))) * 180/pi;

annotation('textbox', [0.15 0.12 0.25 0.07], ...
    'String', sprintf('Mean |Error| = %.3f deg/s', err_abs_mean), ...
    'FitBoxToText', 'on', 'BackgroundColor', 'white', 'EdgeColor', 'black');