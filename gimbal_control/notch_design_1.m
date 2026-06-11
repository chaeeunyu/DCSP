%% ============================================================
%  Notch Filter Design from FFT  (Prewarping + Tustin)
%  - designation_data 의 응답을 FFT 해서 공진 주파수를 찾고
%  - 그 주파수를 제거하는 노치 필터를 prewarping 으로 이산화
%  - C 코드에 넣을 계수를 출력
% ============================================================
clear; clc; close all;

%% --- 0) 설정 ---------------------------------------------------
fname = 'designation_data/dsg_psi-1deg.out';   % <-- 분석할 파일명 수정
Fs    = 200;            % 샘플링 주파수 [Hz]  (= SAMPLING_FREQ)
T     = 1/Fs;
zeta  = 0.1;            % 노치 폭/깊이 (작을수록 좁고 깊음)  <-- TUNE
t0    = 0.5;           % 초기 과도구간 제외 시작시간 [s]

%% --- 1) 데이터 로드 (주석/헤더 줄 자동 스킵) -------------------
% dsg 파일 컬럼: Time OmegaCmd Vc Vg Pot Omega Psi  (총 7열)
lines = readlines(fname);
data  = [];
for i = 1:numel(lines)
    v = sscanf(lines(i), '%f');
    if numel(v) == 7            % 숫자 7개인 줄(=데이터)만 채택
        data(end+1,:) = v(:).'; 
    end
end

t   = data(:,1);
om  = data(:,6);     % Omega [rad/s]  (각속도)
psi = data(:,7);     % Psi   [deg]    (각도)

%% --- 2) FFT 분석 ----------------------------------------------
sig  = psi;                       % 분석 대상 (psi 권장, 필요시 om)
mask = t >= t0;
x    = sig(mask);
x    = x - mean(x);               % DC 제거
N    = numel(x);

win  = hann(N);                   % 윈도우 (스펙트럼 누설 감소)
X    = fft(x.*win);
f    = (0:N-1).' * (Fs/N);
mag  = abs(X) / sum(win) * 2;     % 단측 진폭 정규화

half        = 1:floor(N/2);
[~, ipk]    = max(mag(half));
f_notch     = f(ipk);             % 지배적 공진 주파수 [Hz]
fprintf('Dominant (resonance) frequency : %.3f Hz\n', f_notch);

figure('Name','FFT');
plot(f(half), mag(half), 'LineWidth', 1.2); grid on; hold on;
xline(f_notch, 'r--', sprintf('%.2f Hz', f_notch));
xlabel('Frequency [Hz]'); ylabel('|Amplitude|');
title('FFT spectrum of \psi'); xlim([0 50]);

%% --- 3) Notch Filter 설계 (Prewarping + Tustin) ---------------
%  교수님 Prewarping.m 과 동일한 패턴.
%  notch:  H(s) = (s^2 + w0^2) / (s^2 + 2*zeta*w0*s + w0^2)
%          -> 분자 영점이 ±j*w0 (허수축) 이라서 w0 에서 이득이 0 (=노치)
w0 = 2*pi*f_notch;                % [rad/s]

% (a) 아날로그 노치
Hs = tf([1 0 w0^2], [1 2*zeta*w0 w0^2]);

% (b) prewarping : 노치 중심이 정확히 f_notch 에 오도록 보정
wp = (2/T) * tan(w0 * T / 2);
Hs_bar = tf([1 0 wp^2], [1 2*zeta*wp wp^2]);

% (c) Tustin 이산화
Hz = c2d(Hs_bar, T, 'tustin');
[b, a] = tfdata(Hz, 'v');         % b = [b0 b1 b2], a = [a0 a1 a2]

%% --- 4) C 코드용 계수 출력 (a0 정규화) ------------------------
b = b / a(1);  a = a / a(1);      % a0 = 1 로 정규화
fprintf('\n--- C code coefficients (a0 = 1) ---\n');
fprintf('nb0 = %+.10e;\n', b(1));
fprintf('nb1 = %+.10e;\n', b(2));
fprintf('nb2 = %+.10e;\n', b(3));
fprintf('na1 = %+.10e;\n', a(2));
fprintf('na2 = %+.10e;\n', a(3));
fprintf('(C 코드는 NotchInit() 안에서 NOTCH_FREQ, NOTCH_ZETA 로 자동계산하므로\n');
fprintf(' 위 값은 검증용입니다. NOTCH_FREQ = %.2f 로 넣으세요.)\n', f_notch);

%% --- 5) 검증 : prewarping 유무 비교 ---------------------------
figure('Name','Notch Bode');
bode(Hs, 'k');                     hold on;   % 아날로그
bode(c2d(Hs, T, 'tustin'), 'b--');            % prewarping 없이
bode(Hz, 'r');                                % prewarping 적용
grid on;
legend({'Analog','Tustin (no prewarp)','Tustin (prewarp)'}, 'FontSize', 11);
title('Notch filter discretization (prewarping check)');
