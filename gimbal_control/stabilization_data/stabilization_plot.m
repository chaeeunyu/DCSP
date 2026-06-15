clear; close all; clc;

data = readmatrix('stb_Kp1.0288_Ki29.5203.out', 'FileType', 'text', 'NumheaderLines', 5);

time = data(:, 1);
omega_U = data(:, 2);
pot = data(:, 5);
omega_h = data(:, 6);
error = data(:, 7);

omega_h = omega_h * 180/pi;
Kpot = 68.07352; % [deg/V]
angle = Kpot*(pot - 2.5);  % [deg]

figure(1);
grid on; hold on;
plot(time, omega_U);
plot(time, omega_h);
legend('omega_U', 'omega_h');
xlabel('time [sec]'); ylabel('omega U [deg/s]');
title('Omega_U');

figure(2);
grid on; hold on;
plot(time, angle);
xlabel('time [sec]'); ylabel('angle [deg]');
title('potentio');

figure(3);
grid on; hold on;
plot(time, error);
xlabel('time [sec]'); ylabel('error [rad/s]');
title('error = Wc - Wh');