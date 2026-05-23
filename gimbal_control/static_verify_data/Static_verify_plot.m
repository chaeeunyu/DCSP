%% Static Verify – omega_avg vs omega_target 비교
clear; clc; close all;

%% ── 0. 파일 읽기 ──────────────────────────────────────────────────────
fname = 'summary.out';

raw          = readmatrix(fname, 'CommentStyle','%', 'FileType','text');
Vcmd         = raw(:,2);
omega_avg    = raw(:,4);
omega_target = raw(:,5);   % = K_LIN * Vcmd

err = omega_avg - omega_target;

%% ── 1. CW / CCW 인덱스 & 선형 회귀 기울기 ────────────────────────────
cw  = Vcmd > 0;
ccw = Vcmd < 0;

% 실측 기울기 (원점 통과 강제 선형 회귀)
slope_cw_meas  = Vcmd(cw)  \ omega_avg(cw);
slope_ccw_meas = Vcmd(ccw) \ omega_avg(ccw);

% omega_target 기울기 (= K_LIN, 원점 통과)
slope_target = Vcmd(cw) \ omega_target(cw);   % CW/CCW 동일 = K_LIN

% 회귀선용 x 벡터
x_cw  = linspace(0,    2.5, 100)';
x_ccw = linspace(-2.5, 0,   100)';

%% ── 2. Figure : CW + CCW 한 plot ────────────────────────────────────
figure(1); clf;
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

% ── 상단: 특성 곡선 + 기울기 ─────────────────────────────────────────
nexttile;
hold on; grid on; box on;

% omega_target (점선)
plot(Vcmd(cw),  omega_target(cw),  'r--', 'LineWidth',1.8, 'HandleVisibility','off');
plot(Vcmd(ccw), omega_target(ccw), 'r--', 'LineWidth',1.8, 'DisplayName', ...
     sprintf('\\omega_{target}  (slope = %.4f)', slope_target));

% 실측 데이터 (마커)
plot(Vcmd(cw),  omega_avg(cw),  'b.-', 'MarkerSize',11, 'LineWidth',1.2, 'HandleVisibility','off');
plot(Vcmd(ccw), omega_avg(ccw), 'b.-', 'MarkerSize',11, 'LineWidth',1.2, 'HandleVisibility','off');

% 실측 회귀선 (실선)
plot(x_cw,  slope_cw_meas  * x_cw,  'b-',  'LineWidth',2.0, 'DisplayName', ...
     sprintf('\\omega_{meas} CW  fit  (slope = %.4f)', slope_cw_meas));
plot(x_ccw, slope_ccw_meas * x_ccw, 'b--', 'LineWidth',2.0, 'DisplayName', ...
     sprintf('\\omega_{meas} CCW fit  (slope = %.4f)', slope_ccw_meas));

xline(0,'k:','LineWidth',1,'HandleVisibility','off');
yline(0,'k:','LineWidth',1,'HandleVisibility','off');

xlabel('V_{cmd} [V]',   'FontSize',12);
ylabel('\omega [rad/s]', 'FontSize',12);
title('Static Characteristic  –  CW & CCW', 'FontSize',13);
legend('Location','northwest','FontSize',10);

% 기울기 텍스트 박스
str = sprintf('K_{lin}  = %.4f  (target)\nK_{CW}   = %.4f  (meas)\nK_{CCW}  = %.4f  (meas)', ...
              slope_target, slope_cw_meas, slope_ccw_meas);
text(0.02, 0.97, str, 'Units','normalized', 'VerticalAlignment','top', ...
     'FontSize',10, 'FontName','FixedWidth', ...
     'BackgroundColor','white', 'EdgeColor',[0.6 0.6 0.6]);

% ── 하단: 오차 ───────────────────────────────────────────────────────
nexttile;
hold on; grid on; box on;

bar(Vcmd(cw),  err(cw),  'FaceColor',[0.25 0.55 0.9], 'EdgeColor','none', 'DisplayName','CW');
bar(Vcmd(ccw), err(ccw), 'FaceColor',[0.95 0.45 0.1], 'EdgeColor','none', 'DisplayName','CCW');
yline(0,'k-','LineWidth',1,'HandleVisibility','off');

xlabel('V_{cmd} [V]',         'FontSize',12);
ylabel('\Delta\omega [rad/s]', 'FontSize',12);
title('\omega_{meas} - \omega_{target}  (Absolute Error)', 'FontSize',13);
legend('Location','best','FontSize',10);

sgtitle('Static Linearization Verify', 'FontSize',14,'FontWeight','bold');

%% ── 3. 터미널 요약 ───────────────────────────────────────────────────
fprintf('\n====================================================\n');
fprintf('  Slope (원점 통과 선형 회귀)\n');
fprintf('  K_lin   (target) : %.4f\n', slope_target);
fprintf('  K_CW    (meas)   : %.4f  (오차 %+.2f%%)\n', ...
        slope_cw_meas,  (slope_cw_meas  - slope_target)/slope_target*100);
fprintf('  K_CCW   (meas)   : %.4f  (오차 %+.2f%%)\n', ...
        slope_ccw_meas, (slope_ccw_meas - slope_target)/slope_target*100);
fprintf('----------------------------------------------------\n');
fprintf('  RMS error (전체) : %.4f rad/s\n', sqrt(mean(err.^2)));
fprintf('  RMS error (CW)   : %.4f rad/s\n', sqrt(mean(err(cw).^2)));
fprintf('  RMS error (CCW)  : %.4f rad/s\n', sqrt(mean(err(ccw).^2)));
fprintf('  Max |error|      : %.4f rad/s  @ Vcmd = %+.2f V\n', ...
        max(abs(err)), Vcmd(abs(err)==max(abs(err))));
fprintf('====================================================\n\n');