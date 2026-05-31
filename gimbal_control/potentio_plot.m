clear; close all; clc;

data = readmatrix('sine_A400deg_F0.0250.out', 'FileType', 'text', 'NumHeaderLines', 2);

time = data(:, 1);
omega = data(:, 6);
pot = data(:, 5);

figure(1);
grid on; hold on;
plot(time, pot);
xlabel('time[sec]'); ylabel('potentiometer [V]');
xlim([0, 40]);

figure(2);
grid on; hold on;
plot(time, omega);
