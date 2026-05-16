clear all, close all, clc;

data = readmatrix('signal3_C_visualCode.txt');

time = data(:, 1);
Vcmd = data(:, 2);
xk = data(:, 3);
yk = data(:, 4);

% figure(1); % 시간 
% plot(time, Vcmd);
% grid on; box on;
% xlabel('time[s]'), ylabel('Vcmd[V]');
% 
% figure(2);
% plot(time, xk);
% grid on; box on;
% xlabel('time[s]'), ylabel('x[k][V]');

figure(3);
% plot(time, yk);
% grid on; box on;
% xlabel('time[s]'), ylabel('y[k][V]');

stem(time, yk, 'filled', 'linewidth', 1);
xlim([0 0.2]); % 원하는 구간 확대
ylim([2 4.5]); 
xlabel('time[s]'), ylabel('Vcmd[V]');
title('Node 3 with capacitor (visual studio data)')
grid on;
