clear; close all; clc;

%% 데이터 로드
D = readmatrix('pot_record_cw60.out', ...
    'FileType', 'text', 'NumHeaderLines', 5);
time = D(:, 1);
V = D(:, 3);
theta = 68.07*(V-2.511);

figure;
plot(time, theta, 'b', 'LineWidth', 1.3);

ylabel('theta');
xlabel('Time [s]');
grid on;