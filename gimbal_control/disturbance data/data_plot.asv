clear; close all; clc;

%% ── 0. Load data ──────────────────────────────
d = readmatrix('stb_Kp1.0288_Ki29.5203_latest.out', 'Filetype', 'text', 'NumHeaderLines', 5);

t= d(:, 1);
disturbance = d(:, 6);
figure(1);
plot(t, disturbance, 'b', 'LineWidth', 1);

grid on;


