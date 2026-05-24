clear; clc; close all;

%% ================================================================
%  0. 데이터 로드
%% ================================================================
data_dir = 'C:\Users\ADMIN\source\repos\DCSP\gimbal_control\motor_sweep_data';
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
        Vc_CW(end+1) = vcmd;  om_CW(end+1) = omega_mean;
    else
        Vc_CCW(end+1) = vcmd; om_CCW(end+1) = omega_mean;
    end
end

[Vc_CW,  idx] = sort(Vc_CW);  om_CW  = om_CW(idx);
[Vc_CCW, idx] = sort(Vc_CCW); om_CCW = om_CCW(idx);

%% ================================================================
%  1. ★ 파라미터 — 여기만 수정
%% ================================================================
Vcmd_dead      = 0.05;   % 데드존 경계 [V]
Vcmd_max       = 2.5;    % 최대 명령 전압
Vc_fit_min     = 1.5;    % CCW 포화 전압
Vc_fit_max     = 3.5;    % CW  포화 전압
om_fit_thresh  = 0.5;    % ★ 피팅용 필터 (데드존 경계 데이터 제거)

%% ================================================================
%  2. 피팅 & K 결정
%% ================================================================
valid_CW  = om_CW  >  om_fit_thresh & Vc_CW  >= Vc_fit_min & Vc_CW  <= Vc_fit_max;
valid_CCW = om_CCW < -om_fit_thresh & Vc_CCW >= Vc_fit_min & Vc_CCW <= Vc_fit_max;

p_cw  = polyfit(Vc_CW(valid_CW),   om_CW(valid_CW),   2);
p_ccw = polyfit(Vc_CCW(valid_CCW), om_CCW(valid_CCW), 2);

omega_sat = max( abs(polyval(p_cw, Vc_fit_max)), abs(polyval(p_ccw, Vc_fit_min)) );
K = omega_sat / Vcmd_max;

fprintf('[CW  피팅] %.4f*Vc^2 + %.4f*Vc + %.4f\n', p_cw(1),  p_cw(2),  p_cw(3));
fprintf('[CCW 피팅] %.4f*Vc^2 + %.4f*Vc + %.4f\n', p_ccw(1), p_ccw(2), p_ccw(3));
fprintf('K = %.4f (rad/s)/V,  omega_sat = %.2f rad/s\n', K, omega_sat);
%% ================================================================
%  3. Figure 1: 산점도 + 피팅 곡선
%% ================================================================
figure(1); clf; hold on; grid on;
scatter(Vc_CW,  om_CW,  60, 'b', 'filled', 'DisplayName', 'CW 데이터');
scatter(Vc_CCW, om_CCW, 60, 'r', 'filled', 'DisplayName', 'CCW 데이터');

Vc_line_cw  = linspace(min(Vc_CW(valid_CW)),   max(Vc_CW(valid_CW)),  200);
Vc_line_ccw = linspace(min(Vc_CCW(valid_CCW)), max(Vc_CCW(valid_CCW)), 200);
plot(Vc_line_cw,  polyval(p_cw,  Vc_line_cw),  'm-', 'LineWidth', 2, 'DisplayName', 'CW 피팅');
plot(Vc_line_ccw, polyval(p_ccw, Vc_line_ccw), 'g-', 'LineWidth', 2, 'DisplayName', 'CCW 피팅');

yline(0, 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off');
xlabel('V_c [V]', 'FontSize', 13);
ylabel('\omega [rad/s]', 'FontSize', 13);
title('V_c vs \omega  (모터 특성 곡선)', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 11);
xlim([0 5]); ylim([-30 30]);
set(gcf, 'Position', [100, 100, 900, 550]);

%% ================================================================
%  4. 역매핑 (Vcmd → Vc)
%% ================================================================
Vcmd_ref  = -Vcmd_max : 0.001 : Vcmd_max;
Vc_mapped = zeros(size(Vcmd_ref));

for i = 1:length(Vcmd_ref)
    vc = Vcmd_ref(i);

    if abs(vc) <= Vcmd_dead
        Vc_mapped(i) = 2.5;   % 데드존: 중립 (보간은 여기 수정)

    elseif vc > Vcmd_dead
        % CW 역함수
        om = K * vc;
        a = p_cw(1); b = p_cw(2);
        disc = b^2 - 4*a*(p_cw(3) - om);
        if disc >= 0
            roots_ = [(-b + sqrt(disc)), (-b - sqrt(disc))] / (2*a);
            valid  = roots_(roots_ >= Vc_fit_min & roots_ <= Vc_fit_max);
            if ~isempty(valid), Vc_mapped(i) = min(valid);
            else,               Vc_mapped(i) = Vc_fit_max; end
        else
            Vc_mapped(i) = Vc_fit_max;
        end

    else
        % CCW 역함수
        om = K * vc;
        a = p_ccw(1); b = p_ccw(2);
        disc = b^2 - 4*a*(p_ccw(3) - om);
        if disc >= 0
            roots_ = [(-b + sqrt(disc)), (-b - sqrt(disc))] / (2*a);
            valid  = roots_(roots_ >= Vc_fit_min & roots_ <= Vc_fit_max);
            if ~isempty(valid), Vc_mapped(i) = min(valid);
            else,               Vc_mapped(i) = Vc_fit_min; end
        else
            Vc_mapped(i) = Vc_fit_min;
        end
    end
end

Vc_mapped = max(Vc_fit_min, min(Vc_fit_max, Vc_mapped));

%% ================================================================
%  5. Figure 2: 선형화 검증
%% ================================================================
omega_final = zeros(size(Vcmd_ref));
for i = 1:length(Vcmd_ref)
    if     Vcmd_ref(i) >  Vcmd_dead, omega_final(i) = polyval(p_cw,  Vc_mapped(i));
    elseif Vcmd_ref(i) < -Vcmd_dead, omega_final(i) = polyval(p_ccw, Vc_mapped(i));
    end
end

figure(2); clf; hold on; grid on;
plot(Vcmd_ref, K*Vcmd_ref,  'k--', 'LineWidth', 1.5, 'DisplayName', '목표: \omega = K·V_{cmd}');
plot(Vcmd_ref, omega_final, 'r-',  'LineWidth', 2,   'DisplayName', '선형화 후 실제 출력');
xline( Vcmd_dead, 'b:', 'LineWidth', 1, 'DisplayName', sprintf('±Vcmd_{dead} = %.2fV', Vcmd_dead));
xline(-Vcmd_dead, 'b:', 'LineWidth', 1, 'HandleVisibility', 'off');
yline(0, 'k:', 'LineWidth', 1, 'HandleVisibility', 'off');
xlabel('V_{cmd,ref} [V]', 'FontSize', 13);
ylabel('\omega [rad/s]', 'FontSize', 13);
title('선형화 검증', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 11);
xlim([-Vcmd_max Vcmd_max]); ylim([-30 30]);
set(gcf, 'Position', [100, 100, 900, 550]);

%% ================================================================
%  6. Figure 3: Lookup Table
%% ================================================================
figure(3); clf; hold on; grid on;
plot(Vcmd_ref, Vc_mapped, 'b-', 'LineWidth', 2, 'DisplayName', 'Lookup Table');
yline(2.5,        'r--', 'LineWidth', 1, 'DisplayName', '중립 2.5V');
yline(Vc_fit_max, 'g--', 'LineWidth', 1, 'DisplayName', sprintf('Vc_{sat} CW = %.1fV',  Vc_fit_max));
yline(Vc_fit_min, 'g--', 'LineWidth', 1, 'DisplayName', sprintf('Vc_{sat} CCW = %.1fV', Vc_fit_min));
xline( Vcmd_dead, 'b:', 'LineWidth', 1, 'DisplayName', sprintf('±Vcmd_{dead} = %.2fV', Vcmd_dead));
xline(-Vcmd_dead, 'b:', 'LineWidth', 1, 'HandleVisibility', 'off');
xlabel('V_{cmd,ref} [V]', 'FontSize', 13);
ylabel('V_c [V]', 'FontSize', 13);
title('역함수 매핑 (Lookup Table)', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 11);
xlim([-Vcmd_max Vcmd_max]); ylim([0 5]);
set(gcf, 'Position', [100, 100, 900, 550]);