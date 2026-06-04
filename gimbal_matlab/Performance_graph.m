% ---------------------------------------------
%     Designation Loop Performance graph
%     - Controller : Velocity Feedback (Inexact D 대체)
% ---------------------------------------------
clear; close all; clc;

% 1. System Parameters
Km = 9.993;
Pm = 10.87;

% 2. Design Parameter Vectors
Wc_vec = 15:0.5:40;
Zc_vec = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9];

% 3. Pre-allocation
tr_matrix  = zeros(length(Zc_vec), length(Wc_vec));
OS_matrix  = zeros(length(Zc_vec), length(Wc_vec));
Ess_matrix = zeros(length(Zc_vec), length(Wc_vec));

s = tf('s');
Gm = Km / (s + Pm);

% 시뮬레이션 시간 - 낮은 damping ratio는 수렴이 느리므로 여유있게
time = 0:0.001:2.0;

% 4. Nested For Loop
figure; hold on; grid on;

for i = 1:length(Zc_vec)
    Zc = Zc_vec(i);

    for j = 1:length(Wc_vec)
        Wc = Wc_vec(j);

        % Gain 계산 (Pole Placement)
        Kp = Wc^2 / Km;
        Kd = (2*Zc*Wc - Pm) / Km;

        % Velocity Feedback 폐루프 전달함수
        % Inner loop: Kd로 rate gyro 피드백
        % Outer loop: Kp로 위치 제어 + 적분기(1/s)
        Go_vel = (Kp * Gm) / (1 + Kd * Gm) * (1/s);
        Gcl    = Go_vel / (1 + Go_vel);

        % DC gain으로 steady-state 값 계산 (dcgain이 step_out(end)보다 정확)
        final_value = dcgain(Gcl);

        % Steady-State Error
        % 단위 step 기준: ess = |1 - final_value|
        Ess_matrix(i,j) = abs(1 - final_value);  % [버그 수정 1] 인덱스 추가

        % Step Response 시뮬레이션
        step_out = step(Gcl, time);

        % Overshoot 계산
        info = stepinfo(Gcl, 'RiseTimeLimits', [0 0.9]);
        OS_matrix(i,j) = info.Overshoot;           % [버그 수정 2] 인덱스 추가

        % tr^90: 0% -> 90% Rising Time
        % [버그 수정 3] min(abs()) 대신 find()로 첫 번째 crossing 검출
        idx_90 = find(step_out >= final_value * 0.9, 1, 'first');

        if isempty(idx_90)
            tr_matrix(i,j) = NaN;  % 수렴 못한 경우
        else
            tr_matrix(i,j) = time(idx_90);
        end
    end

    % 해당 Zc 값에 대한 그래프
    plot(Wc_vec/Pm, tr_matrix(i,:), 'LineWidth', 2, 'DisplayName', sprintf('zeta_c = %.1f', Zc));
end

% 5. Graph Formatting
title('rise time 성능지표');
xlabel('Control Bandwidth, \omega_c/Pm [-]');
% overshoot
% ylabel('Overshoot [%%]');
% yline(10, 'r--', 'Target Spec (%OS < 10%)', ...   % [수정] 0.1 → 10
%       'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
% rise time
ylabel('Rising Time, t_{r}^{90} [sec]');
yline(0.1, 'r--', 'Target Spec (t_r = 0.1s)', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
legend('show', 'Location', 'northeast');
hold off;