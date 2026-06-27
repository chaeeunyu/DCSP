% ------------------------------------------------------
%    pole placement plot - designation (PD-controller)
% ------------------------------------------------------
close all; clear; clc;

Km = 13.55;  % [deg/(s^2*V)]
Pm = 14.06;  % [rad/s or 1/s]

% Design parameter --- PD-controller
Wc = 32;  % [rad/s] <================================ MODIFY
Zc = 0.7;  % [-] <================================ MODIFY

Kp = Wc^2 / Km ;       % unit: [rad/s]
Kd = (2*Zc*Wc - Pm) / Km ;  % unit: [-] dimensionless

% transfer function
s = tf('s');
Gm = Km / (s + Pm);    % input: omega_c [deg/s], output: omega [deg/s]
% Gc = Kp + Kd * s ;
Go_vel = Kp*Gm / (1 + Kd*Gm) * (1/s);
Go_vel = minreal(Go_vel);
Gcl = minreal(Go_vel / (1 + Go_vel));

% --------------------------------- pz map -----------------------------------------
[p_Gm, z_Gm] = pzmap(Gm);
[p_Gcl, z_Gcl] = pzmap(Gcl);
real_pGcl = real(p_Gcl(1)); imag_pGcl = imag(p_Gcl(1)); % bc of complex conjugated poles
real_pGm = real(p_Gm(1)); imag_pGm = imag(p_Gm(1)); % bc of complex conjugated poles

% spec: %OS < 10[%]
Z10 = 0.5912;
TR_spec = 0.1;   % rise time spec [s]
Wc_min = (1 - 0.4167*Z10 + 2.917*Z10^2) / TR_spec;  % = 17.73
wc_max = 40;

% 7. 원하는 Zc, Wc 입력 → pole 위치 표시
Zc_user = 0.7;   % ← 여기 수정
Wc_user = 32;    % ← 여기 수정

% 3. s-평면 격자 그리기를 위한 영역 설정
real_axis = -30:0.1:0;
imag_axis = 0:0.1:30;
[R, I] = meshgrid(real_axis, imag_axis);
S = R + 1i*I;

% 물리적 관계식 역산: omega_c = abs(s), zeta_c = -real(s)/abs(s)
Wc_grid = abs(S);
Zeta_grid = -real(S) ./ Wc_grid;
Zeta_grid(Wc_grid == 0) = 1; % 원점 예외 처리

% 4. 스펙 만족 영역 조건 생성 (Masking)
acceptable_region = (Zeta_grid >= Z10) & (Wc_grid >= Wc_min) & (Wc_grid <= wc_max);

% 5. 플롯 시작
figure('Color', [1 1 1]);
hold on;

% 스펙 만족 영역 색칠 (Acceptable Area)
contourf(R, I, double(acceptable_region), [0.5 0.5], 'FaceColor', [0.8 0.95 0.8], 'EdgeColor', 'none', 'HandleVisibility', 'off');
contourf(R, -I, double(acceptable_region), [0.5 0.5], 'FaceColor', [0.8 0.95 0.8], 'EdgeColor', 'none', 'HandleVisibility', 'off');

% 경계선 강조 플롯 (Boundary Lines)
theta_limit = acos(Z10);
r_lines = 0:0.5:40;
plot(-r_lines*cos(theta_limit),  r_lines*sin(theta_limit), 'r--', 'LineWidth', 1.5, 'DisplayName', '%OS boundary (Zc=0.6)');
plot(-r_lines*cos(theta_limit), -r_lines*sin(theta_limit), 'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');

% [수정] arc 각도 범위: LHP 내 zeta 경계선 → 음의 실축
th = linspace(pi - theta_limit, pi, 100);
plot(Wc_min*cos(th),  Wc_min*sin(th), 'b:',  'LineWidth', 1.5, 'DisplayName', 'Min \omega_c = 18[rad/s]');
plot(Wc_min*cos(th), -Wc_min*sin(th), 'b:',  'LineWidth', 1.5, 'HandleVisibility', 'off');
plot(wc_max*cos(th),  wc_max*sin(th), 'm-.', 'LineWidth', 1.5, 'DisplayName', 'Max \omega_c = 40[rad/s]');
plot(wc_max*cos(th), -wc_max*sin(th), 'm-.', 'LineWidth', 1.5, 'HandleVisibility', 'off');

% 6. 극점(Poles) 이동 플롯
% Wc_test = 15:1:25;
% Zc_test = 0.6;
% 
% for k = 1:length(Wc_test)
%     Kp = Wc_test(k)^2 / Km;
%     Kd = (2*Zc_test*Wc_test(k) - Pm) / Km;
%     poles = roots([1, (Pm + Km*Kd), Km*Kp]);
% 
%     % [수정] HandleVisibility는 문자열 'on'/'off' 사용
%     if k == 1
%         plot(real(poles), imag(poles), 'kx', 'MarkerSize', 8, 'LineWidth', 2, ...
%             'DisplayName', 'Closed-loop poles');
%     else
%         plot(real(poles), imag(poles), 'kx', 'MarkerSize', 8, 'LineWidth', 2, ...
%             'HandleVisibility', 'off');
%     end
% end

sigma_d = -Zc_user * Wc_user;
omega_d =  Wc_user * sqrt(1 - Zc_user^2);

% plot([sigma_d sigma_d], [omega_d -omega_d], 'b pentagram', ...
%     'MarkerSize', 6, 'LineWidth', 3, 'DisplayName', sprintf('Selected Zc=%.1f, Wc=%.0f', Zc_user, Wc_user));

% 그래프 포맷팅
grid on;
xline(0, 'k-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
yline(0, 'k-', 'HandleVisibility', 'off');
xlabel('Real Axis (\sigma)', 'FontSize', 11);
ylabel('Imaginary Axis (j\omega_d)', 'FontSize', 11);
title('Designation Loop Pole Placement Specification Window', 'FontSize', 12);
axis([-30 5 -25 25]);
legend('Location', 'southeast');

% plot pz of Gm & Gcl
plot(real(p_Gm),  imag(p_Gm),  'kx', 'MarkerSize', 12, 'LineWidth', 2, ...
    'DisplayName', sprintf('Gm pole (\\sigma=%.2f)', real(p_Gm(1))));
plot(real(p_Gcl), imag(p_Gcl), 'bx', 'MarkerSize', 12, 'LineWidth', 2, ...
    'DisplayName', sprintf('Gcl poles (\\sigma=%.2f, \\omega_d=%.2f)', real(p_Gcl(1)), abs(imag(p_Gcl(1)))));
if ~isempty(z_Gcl)
    real_zGcl = real(z_Gcl(1)); imag_zGcl = imag(z_Gcl(1));
end
if ~isempty(z_Gm)
    real_zGm = real(z_Gm(1)); imag_zGm = imag(z_Gm(1));
end

hold off;



