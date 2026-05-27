clear; clc; close all;

%% ================================================================
%  0. 데이터 로드
%% ================================================================
data_dir = 'motor_sweep_data';
files = dir(fullfile(data_dir, 'step_*.out'));
files = sort({files.name});
files = files(~cellfun(@isempty, regexp(files, 'step_\d{3}_')));

Vc_CW = []; om_CW = [];
Vc_CCW = []; om_CCW = [];

for i = 1:numel(files)
    tok = regexp(files{i}, 'step_\d+_V(\d+)\.(\d+)_(CW|CCW)\.out', 'tokens');
    if isempty(tok), continue; end
    vcmd = str2double(tok{1}{1}) + str2double(tok{1}{2}) * 0.01;
    dir_ = tok{1}{3};

    fid = fopen(fullfile(data_dir, files{i}), 'r');
    raw = textscan(fid, '%f %f %f %f %f', 'HeaderLines', 8, 'CommentStyle', '%');
    fclose(fid);
    if isempty(raw{1}), continue; end

    t = raw{1}; omega = raw{5};
    omega_mean = mean(omega(t >= t(end) - 2.0));

    if strcmp(dir_, 'CW')
        Vc_CW(end+1)  = vcmd;  om_CW(end+1)  = omega_mean * (180/pi); % rad/s → deg/s
    else
        Vc_CCW(end+1) = vcmd;  om_CCW(end+1) = omega_mean * (180/pi); % rad/s → deg/s
    end
end

[Vc_CW,  idx] = sort(Vc_CW);  om_CW  = om_CW(idx);
[Vc_CCW, idx] = sort(Vc_CCW); om_CCW = om_CCW(idx);

%% ================================================================
%  1. 파라미터 — 여기만 수정
%% ================================================================
Wc_DZ      = 20.0;    % 데드존 경계  [deg/s]
Wc_SAT     = 1400.0;  % 포화 속도    [deg/s]  (데이터에 맞게 조정)
om_fit_min = 30.0;    % 피팅 범위 하한 [deg/s]  (데드존 근처 노이즈 제거)
om_fit_max = 1500.0;  % 피팅 범위 상한 [deg/s]

%% ================================================================
%  2. 피팅 — ★ 축 교환: x = omega, y = Vc  /  4차 다항식
%% ================================================================
valid_CW  = om_CW  >  om_fit_min & om_CW  <  om_fit_max;
valid_CCW = om_CCW < -om_fit_min & om_CCW > -om_fit_max;

% polyfit( x=omega [deg/s],  y=Vc,  차수=4 )  → Vc = f(omega) 직접 피팅
CoefP = polyfit(om_CW(valid_CW),   Vc_CW(valid_CW),   4);
CoefN = polyfit(om_CCW(valid_CCW), Vc_CCW(valid_CCW), 4);

fprintf('============================================================\n');
fprintf('[CW  CoefP] ');  fprintf('%+.6e  ', CoefP);  fprintf('\n');
fprintf('[CCW CoefN] ');  fprintf('%+.6e  ', CoefN);  fprintf('\n');
fprintf('============================================================\n');

%% ================================================================
%  3. Figure 1: 정적 특성 산점도 + 피팅 곡선  (x=omega [deg/s], y=Vc)
%% ================================================================
figure(1); clf; hold on; grid on;

scatter(om_CW,  Vc_CW,  50, 'b', 'filled', 'DisplayName', 'CW 데이터');
scatter(om_CCW, Vc_CCW, 50, 'r', 'filled', 'DisplayName', 'CCW 데이터');

om_line_cw  = linspace(om_fit_min,  om_fit_max,  300);
om_line_ccw = linspace(-om_fit_max, -om_fit_min, 300);
plot(om_line_cw,  polyval(CoefP, om_line_cw),  'm-', 'LineWidth', 2, 'DisplayName', 'CW 피팅 (4차)');
plot(om_line_ccw, polyval(CoefN, om_line_ccw), 'g-', 'LineWidth', 2, 'DisplayName', 'CCW 피팅 (4차)');

xline(0,   'k:',  'LineWidth', 0.8, 'HandleVisibility', 'off');
yline(2.5, 'k--', 'LineWidth', 1.0, 'DisplayName', '중립 2.5 V');
xlabel('\omega [deg/s]', 'FontSize', 13);
ylabel('V_c [V]',        'FontSize', 13);
title('정적 특성: V_m = g(\omega_{out})', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 11);
set(gcf, 'Position', [100, 100, 900, 550]);

%% ================================================================
%  4. 선형화 함수: ωc [deg/s] → Vcmd
%     ★ 데드존 구간은 경계 두 점을 직선으로 연결
%% ================================================================
Wcmd_ref = linspace(-Wc_SAT, Wc_SAT, 3001);
Vcmd_out = zeros(size(Wcmd_ref));

% 데드존 경계값 (직선 보간용 양 끝점)
Vc_at_pos_DZ = polyval(CoefP,  Wc_DZ);   % +Wc_DZ 에서 CW 피팅값
Vc_at_neg_DZ = polyval(CoefN, -Wc_DZ);   % -Wc_DZ 에서 CCW 피팅값

fprintf('데드존 경계값: Vc(+DZ) = %.4f V,  Vc(-DZ) = %.4f V\n', ...
        Vc_at_pos_DZ, Vc_at_neg_DZ);

for i = 1:length(Wcmd_ref)
    Wc = Wcmd_ref(i);

    if Wc >= Wc_DZ                        % CW 구간
        Wc_clamp    = min(Wc, Wc_SAT);
        Vcmd_out(i) = polyval(CoefP, Wc_clamp);

    elseif Wc <= -Wc_DZ                   % CCW 구간
        Wc_clamp    = max(Wc, -Wc_SAT);
        Vcmd_out(i) = polyval(CoefN, Wc_clamp);

    else                                   % 데드존: 직선 보간
        % alpha=0 → -Wc_DZ (Vc_at_neg_DZ),  alpha=1 → +Wc_DZ (Vc_at_pos_DZ)
        alpha       = (Wc - (-Wc_DZ)) / (2 * Wc_DZ);
        Vcmd_out(i) = Vc_at_neg_DZ + alpha * (Vc_at_pos_DZ - Vc_at_neg_DZ);
    end
end

%% ================================================================
%  5. 검증: interp1 로 실제 출력 추정  (교수님 방식)
%% ================================================================
Vm_all = [Vc_CCW, Vc_CW];
Wg_all = [om_CCW, om_CW];          % 이미 deg/s
[Vm_all, sortIdx] = sort(Vm_all);
Wg_all = Wg_all(sortIdx);

Wout = zeros(size(Wcmd_ref));
for i = 1:length(Wcmd_ref)
    if Vcmd_out(i) >= min(Vm_all) && Vcmd_out(i) <= max(Vm_all)
        Wout(i) = interp1(Vm_all, Wg_all, Vcmd_out(i), 'linear');
    else
        Wout(i) = interp1(Vm_all, Wg_all, Vcmd_out(i), 'linear', 'extrap');
    end
end

%% ================================================================
%  6. Figure 2: 선형화 함수  V_m = f(ω_c)
%% ================================================================
figure(2); clf; hold on; grid on;

plot(Wg_all, Vm_all, 'b-', 'LineWidth', 1.5, 'DisplayName', ...
     '정적 특성: V_m = g(\omega_{out})');
plot(Wcmd_ref, Vcmd_out, 'r-', 'LineWidth', 2, 'DisplayName', ...
     '선형화 함수: V_m = f^{-1}(\omega_c)');

xline( Wc_DZ, 'b:', 'LineWidth', 1, 'DisplayName', ...
       sprintf('\\pm W_{DZ} = %.0f deg/s', Wc_DZ));
xline(-Wc_DZ, 'b:', 'LineWidth', 1, 'HandleVisibility', 'off');
yline(2.5, 'k--', 'LineWidth', 1, 'DisplayName', '중립 2.5 V');

xlabel('\omega_c [deg/s]', 'FontSize', 13);
ylabel('V_{cmd} [V]',      'FontSize', 13);
title('Linearization Function Design:  V_m = f(\omega_c)', ...
      'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 11);
xlim([-Wc_SAT Wc_SAT]); ylim([0 5]);
set(gcf, 'Position', [100, 100, 900, 550]);

%% ================================================================
%  7. Figure 3: 선형화 검증  ω_c vs ω_out
%% ================================================================
figure(3); clf; hold on; grid on;

plot(Wcmd_ref, Wcmd_ref, 'k--', 'LineWidth', 1.5, 'DisplayName', ...
     '목표: \omega_{out} = \omega_c  (1:1)');
plot(Wcmd_ref, Wout, 'r-', 'LineWidth', 2, 'DisplayName', ...
     '\omega_c \rightarrow f^{-1} \rightarrow V_m \rightarrow motor \rightarrow \omega_{out}');

xline( Wc_DZ, 'b:', 'LineWidth', 1, 'DisplayName', ...
       sprintf('\\pm W_{DZ} = %.0f deg/s', Wc_DZ));
xline(-Wc_DZ, 'b:', 'LineWidth', 1, 'HandleVisibility', 'off');
yline(0, 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off');

xlabel('\omega_c [deg/s]',    'FontSize', 13);
ylabel('\omega_{out} [deg/s]', 'FontSize', 13);
title('Expected Linearization Result: \omega_c vs \omega_{out}', ...
      'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 11);
xlim([-Wc_SAT Wc_SAT]); ylim([-Wc_SAT Wc_SAT]);
set(gcf, 'Position', [100, 100, 900, 550]);