clear; close all; clc;
%% 1. 데이터 로드
D = readmatrix('stb_Kp1.0288_Ki29.5203_latest.out', ...
    'FileType', 'text', 'NumHeaderLines', 5);
time         = D(:,1);
err          = D(:,7);     % Error[deg/s]
disturbance  = D(:,8);
disturbance_omega = (disturbance-1.339600)*29.5203*(180/pi);

figure(1);
plot(time, disturbance, 'b', 'LineWidth', 1.5);
title('Raw disturbance');

%% 2. 샘플링 주파수 계산
dt = mean(diff(time));     % 평균 샘플링 간격
Fs = 1/dt;                  % 샘플링 주파수 [Hz]
fprintf('샘플링 주파수 Fs = %.3f Hz\n', Fs);

%% 3. (선택) FFT로 실제 주파수 성분 확인 — 0.5Hz 근처가 맞는지 검증용
N = length(disturbance_omega);
f_axis = (0:N-1)*(Fs/N);
Y = abs(fft(disturbance_omega - mean(disturbance_omega)));

figure(2);
plot(f_axis(1:floor(N/2)), Y(1:floor(N/2)));
xlim([0 5]);
xlabel('Frequency [Hz]'); ylabel('|FFT|');
title('Disturbance\_omega 주파수 스펙트럼');
grid on;

%% 4. Band-pass filter (0.5Hz 주변 성분만 추출)
f_center = 0.5;     % 중심 주파수 [Hz]
bw       = 0.4;     % 대역폭 [Hz] -> 필요에 맞게 조절 (좁히면 더 깨끗한 사인파, 넓히면 응답 빠름)
f_low  = f_center - bw/2;
f_high = f_center + bw/2;

% Nyquist 주파수 넘는지 체크
if f_high >= Fs/2
    error('f_high가 Nyquist 주파수(%.3f Hz)를 초과합니다. bw를 줄이세요.', Fs/2);
end

[b, a] = butter(4, [f_low f_high]/(Fs/2), 'bandpass');
disturbance_bp = filtfilt(b, a, disturbance_omega);   % 양방향 필터 -> 위상지연 없음

figure(3);
plot(time, disturbance_omega, 'Color', [0.7 0.7 0.7]); hold on;
plot(time, disturbance_bp, 'r', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Disturbance \omega [deg/s]');
legend('원본', sprintf('BPF 결과 (%.2f–%.2f Hz)', f_low, f_high));
title('0.5Hz 주변 성분 추출 (Band-pass Filter)');
grid on;