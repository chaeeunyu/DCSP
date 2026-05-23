clear; clc; close all;

%% ================================================================
%  파라미터 설정 (여기서 값을 변경하세요)
%% ================================================================
omega_thresh = 0.2;   % 데드존 임계값 [rad/s]  ← 이 값만 바꾸면 전체 반영

%% ================================================================
%  0. 파일에서 데이터 추출
%% ================================================================
data_dir = 'C:\Users\ADMIN\source\repos\DCSP\gimbal_control\motor_sweep_data';
files = dir(fullfile(data_dir, 'step_*.out'));
files = sort({files.name});
files = files(~cellfun(@isempty, regexp(files, 'step_\d{3}_')));
fprintf('총 파일 수: %d\n', numel(files));

Vcmd_CW  = [];  Omega_CW  = [];
Vcmd_CCW = [];  Omega_CCW = [];

for i = 1:numel(files)
    fname = files{i};
    tok = regexp(fname, 'step_\d+_V(\d+)\.(\d+)_(CW|CCW)\.out', 'tokens');
    if isempty(tok)
        fprintf('Skip (unrecognized): %s\n', fname);
        continue;
    end
    vcmd      = str2double(tok{1}{1}) + str2double(tok{1}{2}) * 0.01;
    direction = tok{1}{3};

    fid = fopen(fullfile(data_dir, fname), 'r');
    raw = textscan(fid, '%f %f %f %f %f', 'HeaderLines', 8, 'CommentStyle', '%');
    fclose(fid);
    t     = raw{1};
    omega = raw{5};

    if isempty(t)
        fprintf('No data: %s\n', fname);
        continue;
    end

    % 마지막 2초 평균
    t_end      = t(end);
    mask       = t >= (t_end - 2.0);
    omega_mean = mean(omega(mask));

    if strcmp(direction, 'CW')
        Vcmd_CW(end+1)  = vcmd;
        Omega_CW(end+1) = omega_mean;
    else
        Vcmd_CCW(end+1)  = vcmd;
        Omega_CCW(end+1) = omega_mean;
    end
end

% Vcmd 기준 정렬
[Vcmd_CW,  idx] = sort(Vcmd_CW);   Omega_CW  = Omega_CW(idx);
[Vcmd_CCW, idx] = sort(Vcmd_CCW);  Omega_CCW = Omega_CCW(idx);
fprintf('CW: %d개, CCW: %d개\n', numel(Vcmd_CW), numel(Vcmd_CCW));

%% ================================================================
%  1. 변수명 통일
%% ================================================================
Vc_CW  = Vcmd_CW(:);   om_CW  = Omega_CW(:);
Vc_CCW = Vcmd_CCW(:);  om_CCW = Omega_CCW(:);

fprintf('CW  데이터: %d개,  omega 범위: [%.2f, %.2f]\n', length(om_CW),  min(om_CW),  max(om_CW));
fprintf('CCW 데이터: %d개,  omega 범위: [%.2f, %.2f]\n', length(om_CCW), min(om_CCW), max(om_CCW));

%% ================================================================
%  2. Figure 1: Vc vs omega (모터 특성 곡선 - 원시 데이터)
%% ================================================================
figure(1); clf; hold on; grid on;
scatter(Vc_CW,  om_CW,  50, 'b', 'filled', 'DisplayName', 'CW  데이터');
scatter(Vc_CCW, om_CCW, 50, 'r', 'filled', 'DisplayName', 'CCW 데이터');
yline(0, 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off');
xlabel('V_c [V]',        'FontSize', 13);
ylabel('\omega [rad/s]', 'FontSize', 13);
title('V_c vs \omega  (모터 특성 곡선)', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 11);
set(gcf, 'Position', [100, 100, 900, 550]);
xlim([0 5]); ylim([-30 30]);

%% ================================================================
%  3. 유효 데이터 추출 (Vc 1.5~3.5V 저속 구간 + 데드존 제외)
%% ================================================================
valid_CW  = (Vc_CW  > 2.5) & (Vc_CW  <= 3.5) & (om_CW  >  omega_thresh);
valid_CCW = (Vc_CCW >= 1.5) & (Vc_CCW <  2.5) & (om_CCW < -omega_thresh);

vc_cw_fit  = Vc_CW(valid_CW);    om_cw_fit  = om_CW(valid_CW);
vc_ccw_fit = Vc_CCW(valid_CCW);  om_ccw_fit = om_CCW(valid_CCW);

fprintf('\nCW  유효(저속 구간 1.5~3.5V): %d개,  Vc 범위: [%.2f, %.2f]\n', ...
    sum(valid_CW),  min(vc_cw_fit),  max(vc_cw_fit));
fprintf('CCW 유효(저속 구간 1.5~3.5V): %d개,  Vc 범위: [%.2f, %.2f]\n', ...
    sum(valid_CCW), min(vc_ccw_fit), max(vc_ccw_fit));

%% ================================================================
%  4. 가중 최소자승 (WLS) 1차 피팅
%     가중치: w = 1 / (|Vc - 2.5| + epsilon)^n
%       - epsilon : 0 나눗셈 방지 (2.5V 바로 위/아래 점 보호)
%       - n=2     : 중립 근처 강한 강조 (필요시 n=3 으로 증가)
%
%     WLS 정규방정식: (X'*W*X) * p = X'*W*y
%       → p = [slope; intercept]
%% ================================================================
epsilon = 0.05;   % 가중치 분모 보호값
n_w     = 1;      % 가중치 지수 (클수록 중립 집중)

% --- CW ---
w_cw  = 1 ./ (abs(vc_cw_fit  - 2.5) + epsilon).^n_w;
X_cw  = [vc_cw_fit(:), ones(length(vc_cw_fit), 1)];
W_cw  = diag(w_cw(:));
p_cw_col = (X_cw' * W_cw * X_cw) \ (X_cw' * W_cw * om_cw_fit(:));
p_cw  = p_cw_col';   % [slope, intercept]

% --- CCW ---
w_ccw = 1 ./ (abs(vc_ccw_fit - 2.5) + epsilon).^n_w;
X_ccw = [vc_ccw_fit(:), ones(length(vc_ccw_fit), 1)];
W_ccw = diag(w_ccw(:));
p_ccw_col = (X_ccw' * W_ccw * X_ccw) \ (X_ccw' * W_ccw * om_ccw_fit(:));
p_ccw = p_ccw_col';

fprintf('\n[CW  WLS 1차 피팅] omega = %.4f * Vc + (%.4f)\n', p_cw(1),  p_cw(2));
fprintf('[CCW WLS 1차 피팅] omega = %.4f * Vc + (%.4f)\n', p_ccw(1), p_ccw(2));

% Figure 1 에 피팅 직선 추가 (유효 데이터 범위만)
Vc_cw_line  = linspace(min(vc_cw_fit),  max(vc_cw_fit),  200);
Vc_ccw_line = linspace(min(vc_ccw_fit), max(vc_ccw_fit), 200);

figure(1);
plot(Vc_cw_line,  polyval(p_cw,  Vc_cw_line),  'm-',  'LineWidth', 2.5, 'DisplayName', 'CW  WLS 피팅');
plot(Vc_ccw_line, polyval(p_ccw, Vc_ccw_line), 'g-',  'LineWidth', 2.5, 'DisplayName', 'CCW WLS 피팅');
legend('Location', 'northwest', 'FontSize', 11);

%% ================================================================
%  5. Figure 4: 가중치 분포 확인
%% ================================================================
figure(4); clf;
subplot(2,1,1);
stem(vc_cw_fit,  w_cw  / max(w_cw),  'b', 'filled', 'MarkerSize', 4);
xlabel('V_c [V]', 'FontSize', 12);  ylabel('정규화 가중치', 'FontSize', 12);
title(sprintf('CW  가중치 분포  (epsilon=%.2f, n=%d)', epsilon, n_w), 'FontSize', 13);
grid on; xlim([2.4 3.6]);

subplot(2,1,2);
stem(vc_ccw_fit, w_ccw / max(w_ccw), 'r', 'filled', 'MarkerSize', 4);
xlabel('V_c [V]', 'FontSize', 12);  ylabel('정규화 가중치', 'FontSize', 12);
title(sprintf('CCW 가중치 분포  (epsilon=%.2f, n=%d)', epsilon, n_w), 'FontSize', 13);
grid on; xlim([1.4 2.6]);
set(gcf, 'Position', [100, 700, 900, 500]);

%% ================================================================
%  6. K 결정 (저속 구간 포화점 Vc_sat = 3.5V / 1.5V 기준)
%% ================================================================
Vc_sat_CW  = 3.5;
Vc_sat_CCW = 5.0 - Vc_sat_CW;
omega_max =  polyval(p_cw,  Vc_sat_CW);
omega_min =  polyval(p_ccw, Vc_sat_CCW);
omega_sat = min(abs(omega_max), abs(omega_min));

Vcmd_max = 2.5;
K = omega_sat / Vcmd_max;

fprintf('\n[K 결정]\n');
fprintf('  Vc_sat_CW  = %.2f V  →  omega_max = %.2f rad/s\n', Vc_sat_CW,  omega_max);
fprintf('  Vc_sat_CCW = %.2f V  →  omega_min = %.2f rad/s\n', Vc_sat_CCW, omega_min);
fprintf('  omega_sat  = %.2f rad/s\n', omega_sat);
fprintf('  K          = %.4f (rad/s)/V\n\n', K);

%% ================================================================
%  7. Inverse Mapping: Vcmd → omega_target → Vc
%     1차 역함수: Vc = (omega - p(2)) / p(1)  ← 단순 나눗셈
%
%     데드존 처리: 양쪽 경계 Vc 를 1차 외삽으로 구한 뒤
%                 Vcmd 에 비례해 선형 보간 → 부드러운 연결 보장
%% ================================================================
Vcmd_ref     = -2.5:0.001:2.5;
omega_target = K * Vcmd_ref;
Vc_mapped    = zeros(size(Vcmd_ref));

% 데드존 경계 Vc 값 (1차 피팅 외삽)
Vc_dead_pos = ( omega_thresh - p_cw(2))  / p_cw(1);   % omega = +omega_thresh → CW 쪽 경계 Vc
Vc_dead_neg = (-omega_thresh - p_ccw(2)) / p_ccw(1);  % omega = -omega_thresh → CCW 쪽 경계 Vc
fprintf('데드존 경계: CW  Vc = %.4f V\n', Vc_dead_pos);
fprintf('데드존 경계: CCW Vc = %.4f V\n', Vc_dead_neg);

% 데드존 내 Vcmd 임계값 (K*Vcmd = ±omega_thresh)
Vcmd_dead = omega_thresh / K;
fprintf('데드존 Vcmd 임계: ±%.4f V\n\n', Vcmd_dead);

for i = 1:length(Vcmd_ref)
    om = omega_target(i);

    if om > omega_thresh
        % CW: 1차 역함수  Vc = (omega - b) / a
        Vc_mapped(i) = (om - p_cw(2)) / p_cw(1);

    elseif om < -omega_thresh
        % CCW: 1차 역함수  Vc = (omega - b) / a
        Vc_mapped(i) = (om - p_ccw(2)) / p_ccw(1);

    else
        Vc_mapped(i) = 2.5;
        % 데드존: CCW 경계 → 중립(2.5V) → CW 경계 선형 보간
        % Vcmd 가 [-Vcmd_dead, +Vcmd_dead] 범위에서 연속적으로 연결
        %t_interp     = (Vcmd_ref(i) + Vcmd_dead) / (2 * Vcmd_dead);  % 0→1
        %Vc_mapped(i) = Vc_dead_neg + (Vc_dead_pos - Vc_dead_neg) * t_interp;
    end
end

% 0~5V 클램핑
Vc_mapped = max(0, min(5, Vc_mapped));

%% ================================================================
%  8. 선형화 검증용 omega 계산
%     (2차 피팅 모델이 없으므로 1차 피팅으로 순방향 계산)
%% ================================================================
omega_final = zeros(size(Vcmd_ref));
for i = 1:length(Vcmd_ref)
    om = K * Vcmd_ref(i);
    if om > omega_thresh
        omega_final(i) = polyval(p_cw,  Vc_mapped(i));
    elseif om < -omega_thresh
        omega_final(i) = polyval(p_ccw, Vc_mapped(i));
    else
        omega_final(i) = 0;   % 데드존 → omega = 0
    end
end

%% ================================================================
%  9. Figure 2: 선형화 검증
%% ================================================================
figure(2); clf; hold on; grid on;
plot(Vcmd_ref, K * Vcmd_ref,  'k--', 'LineWidth', 1.5, ...
    'DisplayName', '목표 직선  \omega = K \cdot V_{cmd}');
plot(Vcmd_ref, omega_final, 'r-',  'LineWidth', 2,   ...
    'DisplayName', 'WLS 선형화 후 실제 출력');
yline(0, 'k:', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(0, 'k:', 'LineWidth', 1, 'HandleVisibility', 'off');
xlabel('V_{cmd,ref} [V]', 'FontSize', 13);
ylabel('\omega [rad/s]',   'FontSize', 13);
title('선형화 검증: V_{cmd} vs \omega  (WLS 1차)', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 11);
xlim([-2.5 2.5]); ylim([-30 30]);
set(gcf, 'Position', [100, 100, 900, 550]);

%% ================================================================
%  10. Figure 3: Lookup Table 시각화
%% ================================================================
figure(3); clf; hold on; grid on;
plot(Vcmd_ref, Vc_mapped, 'b-', 'LineWidth', 2, 'DisplayName', 'Lookup Table (WLS)');
yline(2.5,        'r--', 'LineWidth', 1, 'DisplayName', '중립 2.5V');
xline(0,          'k:', 'LineWidth', 1, 'HandleVisibility', 'off');
yline(Vc_sat_CW,  'g--', 'LineWidth', 1, ...
    'DisplayName', sprintf('Vc\\_sat CW = %.1fV', Vc_sat_CW));
yline(Vc_sat_CCW, 'g--', 'LineWidth', 1, ...
    'DisplayName', sprintf('Vc\\_sat CCW = %.1fV', Vc_sat_CCW));
xlabel('V_{cmd,ref} [V]',  'FontSize', 13);
ylabel('V_c 실제 출력 [V]', 'FontSize', 13);
title('역함수 매핑 테이블 — WLS 1차 선형화', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 11);
xlim([-2.5 2.5]); ylim([0 5]);
set(gcf, 'Position', [100, 100, 900, 550]);

%% ================================================================
%  11. 결과 요약 출력
%% ================================================================
fprintf('============================================================\n');
fprintf('  피팅 방법  : WLS 1차 (가중 최소자승)\n');
fprintf('  피팅 구간  : Vc 1.5~3.5V (저속 중립 근처)\n');
fprintf('  epsilon    = %.3f,  n = %d\n', epsilon, n_w);
fprintf('  omega_thresh = %.4f rad/s\n', omega_thresh);
fprintf('------------------------------------------------------------\n');
fprintf('  CW  피팅   : omega = %.4f * Vc + (%.4f)\n', p_cw(1),  p_cw(2));
fprintf('  CCW 피팅   : omega = %.4f * Vc + (%.4f)\n', p_ccw(1), p_ccw(2));
fprintf('------------------------------------------------------------\n');
fprintf('  K          = %.4f (rad/s)/V\n', K);
fprintf('  omega_sat  = %.2f rad/s\n', omega_sat);
fprintf('  Vc_sat     = %.2f V (CW) / %.2f V (CCW)\n', Vc_sat_CW, Vc_sat_CCW);
fprintf('  Vcmd_max   = %.2f V\n', Vcmd_max);
fprintf('  데드존 경계: Vc_dead_pos=%.4fV, Vc_dead_neg=%.4fV\n', Vc_dead_pos, Vc_dead_neg);
fprintf('============================================================\n');