%% Static Verify – OmegaCmd vs Omega_avg 비교
clear; clc; close all;

%% ── 0. 파일 읽기 ──────────────────────────────────────────────────────
fname = 'summary.out';
raw          = readmatrix(fname, 'CommentStyle','%', 'FileType','text');

omega_cmd    = raw(:,2);       % 명령 각속도 [deg/s]
omega_avg_rad= raw(:,4);       % 실측 평균 각속도 [rad/s]

% 단위 통일을 위해 실측 각속도를 [deg/s]로 변환
omega_avg_deg= omega_avg_rad * (180/pi); 

% 오차 계산 (실측 - 명령) [deg/s]
err = omega_avg_deg - omega_cmd;

%% ── 1. CW / CCW 인덱스 & 선형 회귀 기울기 ────────────────────────────
cw  = omega_cmd > 0;
ccw = omega_cmd < 0;

% 실측 기울기 (원점 통과 강제 선형 회귀) -> 이상적인 추종 시 1.0에 수렴
slope_cw_meas  = omega_cmd(cw)  \ omega_avg_deg(cw);
slope_ccw_meas = omega_cmd(ccw) \ omega_avg_deg(ccw);

% 목표 기울기 (명령 대비 명령이므로 완벽한 1:1 선형 관계)
slope_target   = 1.0; 

%% ── 2. Figure : CW + CCW 한 plot ────────────────────────────────────
figure(1); clf;
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

% ── 상단: 특성 곡선 (\omega_{cmd} vs \omega_{avg}) ───────────────────
nexttile;
hold on; grid on; box on;

% Ideal Reference Line (기울기 = 1 일치선)
plot(omega_cmd(cw),  omega_cmd(cw),  'r--', 'LineWidth',1.8);
plot(omega_cmd(ccw), omega_cmd(ccw), 'r--', 'LineWidth',1.8, 'DisplayName', ...
     sprintf('\\omega_{cmd} (Ideal Slope = %.4f)', slope_target));

% 실측 데이터 (마커 표시)
plot(omega_cmd(cw),  omega_avg_deg(cw),  'g.', 'MarkerSize', 15, 'HandleVisibility','off');
plot(omega_cmd(ccw), omega_avg_deg(ccw), 'g.', 'MarkerSize', 15, 'DisplayName', '\omega_{avg} (Measured)');

xline(0,'k:','LineWidth',1,'HandleVisibility','off');
yline(0,'k:','LineWidth',1,'HandleVisibility','off');
xlabel('\omega_{cmd} [deg/s]',   'FontSize',12);
ylabel('\omega [deg/s]', 'FontSize',12);
title('Static Validation (\omega_{cmd} vs \omega_{avg})', 'FontSize',13);
legend('Location','northwest','FontSize',10);

% ── 하단: 절대 오차 [deg/s] ──────────────────────────────────────────
nexttile;
hold on; grid on; box on;
bar(omega_cmd(cw),  err(cw),  'FaceColor',[0.25 0.55 0.9], 'EdgeColor','none', 'DisplayName','CW');
bar(omega_cmd(ccw), err(ccw), 'FaceColor',[0.95 0.45 0.1], 'EdgeColor','none', 'DisplayName','CCW');

yline(0,'k-','LineWidth',1,'HandleVisibility','off');
xlabel('\omega_{cmd} [deg/s]',         'FontSize',12);
ylabel('\Delta\omega [deg/s]', 'FontSize',12);
title('\omega_{avg} - \omega_{cmd} (Absolute Error)', 'FontSize',13);
legend('Location','best','FontSize',10);

sgtitle('Static Linearization Verify (Cmd vs Avg)', 'FontSize',14,'FontWeight','bold');

%% ── 3. 터미널 요약 출력 ───────────────────────────────────────────────
fprintf('\n====================================================\n');
fprintf('  Gain Slope (명령 [deg/s] 대비 출력 [deg/s] 비율)\n');
fprintf('  Target Slope    : %.4f\n', slope_target);
fprintf('  K_CW  (Measured): %.4f  (오차 %+.2f%%)\n', ...
        slope_cw_meas,  (slope_cw_meas  - slope_target)/slope_target*100);
fprintf('  K_CCW (Measured): %.4f  (오차 %+.2f%%)\n', ...
        slope_ccw_meas, (slope_ccw_meas - slope_target)/slope_target*100);
fprintf('----------------------------------------------------\n');
fprintf('  RMS error (전체) : %.4f deg/s (%.4f rad/s)\n', sqrt(mean(err.^2)), sqrt(mean(err.^2))*pi/180);
fprintf('  RMS error (CW)   : %.4f deg/s\n', sqrt(mean(err(cw).^2)));
fprintf('  RMS error (CCW)  : %.4f deg/s\n', sqrt(mean(err(ccw).^2)));
fprintf('  Max |error|      : %.4f deg/s  @ \\omega_{cmd} = %+.2f deg/s\n', ...
        max(abs(err)), omega_cmd(abs(err)==max(abs(err))));
fprintf('====================================================\n\n');