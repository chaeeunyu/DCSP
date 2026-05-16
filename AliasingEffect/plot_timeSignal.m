clear all, close all, clc;

data = readmatrix('signal_4_C.csv');

time = data(:, 4);
Vout = data(:, 5);

figure;
plot(time, Vout, 'linewidth', 1.5);
grid on; box on;
xlabel('time[s]'), ylabel('Voltage[V]');
title('Node 4 with capacitor');