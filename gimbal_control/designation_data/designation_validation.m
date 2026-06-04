% ──────────────────────────────────────────────
%  Designation Loop : Tracking Performance Plot
%  Wc=24, Zc=0.6
% ──────────────────────────────────────────────
clear; close all; clc;

%% ── 0. Load data ──────────────────────────────
d = readmatrix('dsg_psi-1deg.out', 'Filetype', 'text', 'NumHeaderLines', 5);

t        = d(:,1);   % [s]
omega_c  = d(:,2);   % [deg/s]
Vc       = d(:,3);   % [V]
omega    = d(:,6);   % [rad/s]
psi      = d(:,7);   % [deg]

% header에서 파악한 값
psi_cmd  = -1.0;      % [deg]  <------------------- MODIFY!!
Kp       = 57.6403;
Kd       = 1.7943;
Wc = 27;
Zc = 0.7;

%% ── 1. 위치 추종 ──────────────────────────────
figure(1);
hold on; grid on;
plot(t, psi,                   'b',   'LineWidth', 1.5, 'DisplayName', '\psi (actual)');
plot(t, psi_cmd*ones(size(t)), 'r--', 'LineWidth', 1.2, 'DisplayName', '\psi_{cmd}');
xlabel('Time [s]');
ylabel('\psi [deg]');
title(sprintf('Position Tracking  (Zc=%.1f, Wc=%.f)', Zc, Wc));
legend('Location','best');
xlim([t(1) t(end)]);

str = { sprintf('Kp  = %.3f', Kp)
        sprintf('Kd = %.4f',   Kd)};
annotation('textbox', [0.67 0.7 0.5 0.08], ...
    'String',          str, ...
    'FitBoxToText',    'on', ...
    'BackgroundColor', 'w', ...
    'FontSize',        10);

% % 성능 지표 계산
% e       = psi_cmd - psi;
% Ess     = e(end);
% [~, idx_tr] = min(abs(psi - psi_cmd*0.9 - psi(1)*0.1));   % 90% rise (첫 지점)
% tr90    = t(idx_tr) - t(1);
% [max_os_val, ~] = max(abs(psi - psi_cmd));                 % 피크 편차
% OS      = (max_os_val / abs(psi(1) - psi_cmd)) * 100;     % 초기 오차 대비 %
% 
% str = { sprintf('Ess  = %.3f deg', Ess)
%         sprintf('tr90 = %.4f s',   tr90)
%         sprintf('%%OS  = %.2f %%',  OS)   };
% annotation('textbox','String',str,'FitBoxToText','on','BackgroundColor','w','FontSize',10);
% 
% %% ── 2. 제어 출력 (omega_c, Vc) ───────────────
% figure(2);
% 
% subplot(2,1,1);
% plot(t, omega_c, 'm', 'LineWidth', 1.2);
% hold on; grid on;
% yline( 1400, 'k--', 'WC_{SAT}',  'LabelHorizontalAlignment','left');
% yline(-1400, 'k--', '-WC_{SAT}', 'LabelHorizontalAlignment','left');
% xlabel('Time [s]'); ylabel('\omega_c [deg/s]');
% title('Commanded Angular Rate');
% xlim([t(1) t(end)]);
% 
% subplot(2,1,2);
% plot(t, Vc, 'k', 'LineWidth', 1.2);
% hold on; grid on;
% yline(2.5, 'r--', 'NEUTRAL');
% xlabel('Time [s]'); ylabel('V_c [V]');
% title('Motor Voltage');
% ylim([-0.2 5.2]);
% xlim([t(1) t(end)]);
% 
% %% ── 3. 실측 각속도 ────────────────────────────
% figure(3);
% omega_deg = omega * (180/pi);   % rad/s → deg/s
% plot(t, omega_deg, 'Color',[0.2 0.6 0.2], 'LineWidth', 1.2);
% hold on; grid on;
% yline(0, 'k--');
% xlabel('Time [s]'); ylabel('\omega [deg/s]');
% title('Measured Angular Rate (Gyro)');
% xlim([t(1) t(end)]);