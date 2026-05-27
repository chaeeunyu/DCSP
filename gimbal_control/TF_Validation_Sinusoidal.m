clear; close all; clc;

%% 1. 데이터 로드
D = readmatrix('sine_A400deg_F0.0250.out', ...
    'FileType', 'text', 'NumHeaderLines', 3);

Omega_cmd  = D(:,2);           % [deg/s] (열 벡터)
Omega      = D(:,6) * 180/pi;  % [rad/s] → [deg/s] (열 벡터)

% 💡 원본 시간축 D(:,1)을 버리고, 
% 데이터 개수와 샘플링 주파수(200Hz)를 이용해 오차가 1도 없는 완벽한 시간축 생성
Fs = 200;
t_ideal = (0 : length(Omega_cmd)-1).' / Fs; 

%% 2. 전달함수 시뮬레이션
num = [9.993];
den = [1 10.87];
num2 = [167.8];
den2 = [1 16.62 199.7];
sys = tf(num2, den2);

u_sim_rad = Omega_cmd * (pi/180); 

% 💡 완벽하게 증가하는 t_ideal을 사용하므로 절대 에러가 나지 않습니다.
y_sim_rad = lsim(sys, u_sim_rad, t_ideal);

% 결과를 비교 플롯을 위해 deg/s로 다시 변환
y_sim_deg = y_sim_rad * (180/pi);

%% 3. 주기 및 샘플링 설정 (온전한 주기 추출)
T     = 40;
N_per = T * Fs;                    % 한 주기의 샘플 수 (8000)

len_safe = min([length(t_ideal), length(y_sim_deg)]);
N_cyc    = floor(len_safe / N_per); % 확보 가능한 온전한 주기 수
n_use    = N_per * N_cyc;           % 사용할 총 샘플 수

%% 4. 주기 평균 및 오차 계산 (💡 오차 계산 수식 추가)
% 측정값 평균화
Omega_cmd_m = reshape(Omega_cmd(1:n_use), N_per, N_cyc);
Omega_m     = reshape(Omega(1:n_use),     N_per, N_cyc);
Omega_cmd_avg = mean(Omega_cmd_m, 2);
Omega_avg     = mean(Omega_m,     2);

% 시뮬레이션값 평균화
y_m         = reshape(y_sim_deg(1:n_use), N_per, N_cyc);
y_avg       = mean(y_m, 2);

% 💡 [추가] 정현파 데이터에 대한 실측값과 시뮬레이션 간의 오차 계산
error_avg = Omega_avg - y_avg;
rmse_val  = sqrt(mean(error_avg.^2)); % RMS(평균 제곱근) 오차
max_err   = max(abs(error_avg));      % 최대 절대 오차

% 플롯을 위한 단일 주기 시간 축 생성
t_one = (0 : N_per-1).' / Fs;   % 0 ~ 39.995 s (열 벡터)

%% 5. 플롯 (💡 하단 그래프에 오차 박스 반영)
figure('Position', [100 100 900 650]);

% ── 1) 상단: 명령(Command) 플롯
subplot(2,1,1);
plot(t_one, Omega_cmd_avg, 'b', 'LineWidth', 1.5);
ylabel('\Omega_{cmd} [deg/s]');
title(sprintf('Cycle-Averaged Comparison  (f=0.025 Hz, %d cycles)', N_cyc));
grid on;
xlim([t_one(1) t_one(end)]);

% ── 2) 하단: 실측 데이터 vs 시뮬레이션 플롯 + 오차 텍스트 박스
subplot(2,1,2);
hold on;
plot(t_one, Omega_cmd_avg, 'b--', 'LineWidth', 1.3, 'DisplayName', '\Omega_{cmd} (avg)');
plot(t_one, Omega_avg,     'r-',   'LineWidth', 1.5, 'DisplayName', '\Omega measured (avg)');
plot(t_one, y_avg,         'g-',   'LineWidth', 1.5, 'DisplayName', 'Simulation \omega (tf)'); 
hold off;

ylabel('\Omega [deg/s]');
xlabel('Time [s]');
legend('Location', 'best');
grid on;
xlim([t_one(1) t_one(end)]);

% 💡 [추가] 하단 그래프 좌측 상단 영역에 오차 수치 텍스트 박스 생성
err_str = sprintf('RMS Error : %.2f deg/s\nMax Error : %.2f deg/s', rmse_val, max_err);
text(0.02, 0.95, err_str, 'Units', 'normalized', ...
    'VerticalAlignment', 'top', ...
    'BackgroundColor', 'white', ...
    'EdgeColor', 'black', ...
    'FontSize', 10, ...
    'FontWeight', 'bold');