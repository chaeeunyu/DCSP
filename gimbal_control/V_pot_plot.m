clear; close all; clc;

%% 데이터 로드
D = readmatrix('sine_A400deg_F0.0250.out', ...
    'FileType', 'text', 'NumHeaderLines', 3);

t          = D(:,1);
Pot = D(:, 5);   % [deg/s]
figure(1);

plot(t, Pot, 'b', 'LineWidth', 1.3, 'DisplayName', '\Omega_{cmd} (avg)');
grid on;
