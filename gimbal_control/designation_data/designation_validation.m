% ──────────────────────────────────────────────
%  Designation Loop : Tracking Performance Plot
%  Wc=24, Zc=0.6
% ──────────────────────────────────────────────
clear; close all; clc;

%% ── 0. Load data ──────────────────────────────
d = readmatrix('dsg_psi+20deg_Wc=35.0, Zc=0.7_20260621_131503.out', 'Filetype', 'text', 'NumHeaderLines', 5);
t        = d(:,1);   % [s]
omega_c  = d(:,2);   % [deg/s]
Vc       = d(:,3);   % [V]
omega    = d(:,6);   % [rad/s]
psi      = d(:,7);   % [deg]

% header에서 파악한 값
psi_cmd  = -1.0;      % [deg]  <------------------- MODIFY!!
Kp       = 66.4207;
Kd       = 2.0620;
Wc = 27;
Zc = 0.7;

%% ── 1. 위치 추종 ──────────────────────────────
figure(1);
hold on; grid on;
plot(t, psi,                   'b',   'LineWidth', 1.5, 'DisplayName', '\psi (actual)');
%plot(t, psi_cmd*ones(size(t)), 'r--', 'LineWidth', 1.2, 'DisplayName', '\psi_{cmd}');
xlabel('Time [s]');
ylabel('\psi [deg]');
title(sprintf('Position Tracking  (Zc=%.1f, Wc=%.f)', Zc, Wc));
xlim([t(1) t(end)]);

%% ── 2. Rise time 계산 ──────────────────────────────
final_value   = mean(psi(end-50:end));
initial_value = psi(1);

target90 = initial_value + 0.9 * (final_value - initial_value);
tr_idx   = find(psi >= target90, 1, 'first');
tr       = t(tr_idx);

Ess = psi_cmd - final_value;

%% ── 3. 그래프에 표시 ──────────────────────────────
plot(t(tr_idx), target90, 'r*', 'MarkerSize', 12, 'DisplayName', 'rise time pt');

legend('Location','best');

str = { sprintf('Kp  = %.3f', Kp)
        sprintf('Kd = %.4f',   Kd)};
annotation('textbox', [0.67 0.7 0.5 0.08], ...
'String',          str, ...
'FitBoxToText',    'on', ...
'BackgroundColor', 'w', ...
'FontSize',        10);

str2 = { sprintf('rise time = %.4f [s]', tr)
         sprintf('Ess = %.4f [deg]', Ess) };
annotation('textbox', [0.67 0.55 0.5 0.1], ...
    'String',          str2, ...
    'FitBoxToText',    'on', ...
    'BackgroundColor', 'w', ...
    'FontSize',        10);