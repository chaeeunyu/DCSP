% ---------------------------------------
%     Stabilization Loop Design
%  - Controller : PI
%  - Motor tf : 1st
% ---------------------------------------
%clear; close all; clc;

% initialize
Km = 13.55 ;
Pm = 14.06;
W_sat = 1400;
dz    = 28*(pi/180);
dz_deg = 28;

% disturbance spec frequency (robot body bandwidth ~ 0.5 Hz)
Wmax = 2*pi*0.5;   % [rad/s]  <----------------------------- MODIFY (use given spec)

% Design parameter --- PI-controller
Wc = 21.0;  % <-------------------------------------------- MODIFY
Zc = 0.7;   % <-------------------------------------------- MODIFY

% c코드 stabilization data
% lpf 있는 
D_wLPF = readmatrix('stabilization_500.0[deg_s]_Wc=21.0,Zc=0.7_wLPF.out', ...
    'FileType', 'text', 'NumHeaderLines', 3);
% lpf 없는 
D_woLPF = readmatrix('stabilization_500.0[deg_s]_Wc=21.0,Zc=0.7_noLPF.out', ...
    'FileType', 'text', 'NumHeaderLines', 3);


%  PD : Kp = Wc^2/Km ,        Kd = (2*Zc*Wc - Pm)/Km
%  PI : Ki = Wc^2/Km ,        Kp = (2*Zc*Wc - Pm)/Km   (dual structure)
Ki = Wc^2 / Km ;            % unit: [1/s]   (integral gain)
Kp = (2*Zc*Wc - Pm) / Km ;  % unit: [-]     (proportional gain)

% command magnitudes (read by the Simulink model from the base workspace)
input_deg = 500 ;     % [deg/s]  commanded rate w_c      <--- MODIFY
wb_dist   = 10 ;    % [deg/s]  body-rate disturbance w_b <-- MODIFY

% transfer function
s = tf('s');
Gm = Km / (s + Pm);     % input: omega_c [deg/s], output: omega [deg/s]
Gc = Kp + Ki/s ;        % PI controller

% LPF
fc_lpf   = 20.0;            % [Hz] LPF 컷오프 주파수
Wc_lpf   = 2*pi*fc_lpf;     % [rad/s]
Glpf     = Wc_lpf / (s + Wc_lpf);   % 1st-order LPF

Go  = Gc * Gm * Glpf;               % open loop (rate loop)
Gcl = minreal( Gc*Gm / (1 + Go) );  % command tracking : w_c -> w_h
P   = minreal( -1 / (1 + Go) );  % disturbance tf  : w_b -> e   ( = e/w_b )
Gcl_filt = minreal( Glpf * Gcl );   % omega_lpf와 비교

% disturbance-rejection spec : |e/w_b| at Wmax   (spec : <= 0.1)
P_at_wmax = abs( evalfr(P, 1j*Wmax) );


% ---------------------------- nyquist plot ------------------------------------------
% [GM, PM, Phase crossover freq, Gain Crossover freq]
[GM, PM, wpc, wgc] = margin(Go);
GM_dB = 20*log10(GM);

figure(1); clf;
[re, im, wout] = nyquist(Go);
plot(squeeze(re), squeeze(im));
grid on; hold on;
% plot unit circle
axis([-1.35 1.35 -1.1 1.1]);
ang = 0:0.1:360;
xc = cosd(ang); yc = sind(ang); % [deg]
plot(xc, yc, 'k--');
xline(0, 'k'); yline(0, 'k');
% plot the point (0, -1)
plot(-1, 0, 'p', 'markersize', 10, 'MarkerEdgeColor','r', 'LineWidth', 1);

% plot wpc, wgc
wpc_complex = evalfr(Go, 1j*wpc);
wgc_complex = evalfr(Go, 1j*wgc);
wpc_x = real(wpc_complex); wpc_y = imag(wpc_complex);
wgc_x = real(wgc_complex); wgc_y = imag(wgc_complex);
plot(wpc_x, wpc_y, '*', 'markeredgecolor', 'm', 'MarkerSize', 10, 'LineWidth', 1);
plot(wgc_x, wgc_y, '*', 'markeredgecolor', 'g', 'MarkerSize', 10, 'LineWidth', 1);
legend('', '', '', '', '', sprintf('GM = %.2f [dB] / wpc = %.2f [Hz]', GM_dB, wpc/(2*pi)), ...
    sprintf('PM = %.2f [deg] / wgc = %.2f [Hz]', PM, wgc/(2*pi)));
title('Nyquist Plot, Go(s)');
xlabel('Re\{Go(jw)\} [-]'); ylabel('Im\{Go(jw)\} [-]');

% --------------------------------- pz map -----------------------------------------
[p_Gm, z_Gm] = pzmap(Gm);
[p_Gcl, z_Gcl] = pzmap(Gcl);
real_pGcl = real(p_Gcl(1)); imag_pGcl = imag(p_Gcl(1)); % bc of complex conjugated poles
real_pGm = real(p_Gm(1)); imag_pGm = imag(p_Gm(1)); % bc of complex conjugated poles

figure(2); clf;
% --------------------------------- pole placement spec window -----------------------------------------
% spec: %OS < 10[%]
Z10    = 0.6;
TR_spec = 0.1;
Wc_min  = (1 - 0.4167*Z10 + 2.917*Z10^2) / TR_spec;  % ≈ 17.73
wc_max  = 40;

% s-평면 격자 (Mesh grid)
real_axis = -30:0.1:0;
imag_axis =   0:0.1:30;
[R, I_grid] = meshgrid(real_axis, imag_axis);
S_grid  = R + 1i*I_grid;

Wc_grid   = abs(S_grid);
Zeta_grid = -real(S_grid) ./ Wc_grid;
Zeta_grid(Wc_grid == 0) = 1;

acceptable_region = (Zeta_grid >= Z10) & (Wc_grid >= Wc_min) & (Wc_grid <= wc_max);

figure(2); clf;
hold on;

% 스펙 만족 영역 색칠 (Acceptable Region)
contourf(R,  I_grid, double(acceptable_region), [0.5 0.5], ...
    'FaceColor', [1.0 0.85 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
contourf(R, -I_grid, double(acceptable_region), [0.5 0.5], ...
    'FaceColor', [1.0 0.85 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');

% 경계선 (Boundary Lines)
theta_limit = acos(Z10);
r_lines = 0:0.5:50;
plot(-r_lines*cos(theta_limit),  r_lines*sin(theta_limit), 'r--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('%%OS boundary (Zc=%.1f)', Z10));
plot(-r_lines*cos(theta_limit), -r_lines*sin(theta_limit), 'r--', 'LineWidth', 1.5, ...
    'HandleVisibility', 'off');

th = linspace(pi - theta_limit, pi, 100);
plot(Wc_min*cos(th),  Wc_min*sin(th), 'b:',  'LineWidth', 1.5, ...
    'DisplayName', sprintf('Min \\omega_c = %.1f [rad/s]', Wc_min));
plot(Wc_min*cos(th), -Wc_min*sin(th), 'b:',  'LineWidth', 1.5, 'HandleVisibility', 'off');
plot(wc_max*cos(th),  wc_max*sin(th), 'm-.', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Max \\omega_c = %.0f [rad/s]', wc_max));
plot(wc_max*cos(th), -wc_max*sin(th), 'm-.', 'LineWidth', 1.5, 'HandleVisibility', 'off');

% Gm pole & Gcl poles 오버레이 (Overlay)
plot(real(p_Gm), imag(p_Gm), 'kx', 'MarkerSize', 12, 'LineWidth', 2, ...
    'DisplayName', sprintf('Gm pole (\\sigma=%.2f)', real(p_Gm(1))));
plot(real(p_Gcl), imag(p_Gcl), 'bx', 'MarkerSize', 12, 'LineWidth', 2, ...
    'DisplayName', sprintf('Gcl poles (\\sigma=%.2f, \\omega_d=%.2f)', ...
    real(p_Gcl(1)), abs(imag(p_Gcl(1)))));

% Gcl zero 표시 (PI controller는 zero가 있음)
if ~isempty(z_Gcl)
    plot(real(z_Gcl), imag(z_Gcl), 'bo', 'MarkerSize', 10, 'LineWidth', 2, ...
        'DisplayName', sprintf('Gcl zero (\\sigma=%.2f)', real(z_Gcl(1))));
end

% 축/포맷
grid on;
xline(0, 'k-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
yline(0, 'k-', 'HandleVisibility', 'off');
xlabel('Real Axis (\sigma)', 'FontSize', 11);
ylabel('Imaginary Axis (j\omega_d)', 'FontSize', 11);
title('Stabilization Loop Pole Placement Specification Window', 'FontSize', 12);
axis([-30 5 -25 25]);
legend('Location', 'southeast');
hold off;
grid on; hold on;
% legend(sprintf('Gm: p=%.2f±%.2f[Hz], z=%.2f[Hz]', real_pGm/(2*pi), imag_pGm/(2*pi), z_Gm), ...
%     sprintf('Gcl: p=%.2f±%.2f[Hz], z=%.2f[Hz]\n', real_pGcl/(2*pi), imag_pGcl/(2*pi), z_Gcl));
% ---------------------------------------------------------------------------------

% step input (rate command)
% Tf = 0.5;
% time = 0:0.001:Tf;
% step_input = input_deg * ones(size(time));
% step_out = lsim(Gcl, step_input, time);
% final_value = step_out(end);
% % rise time (90%)
% [~, tr_idx] = min(abs(step_out - final_value*0.9));
% tr = time(tr_idx);
% % overshoot
% [~, os_idx] = max(step_out);
% max_value = step_out(os_idx);
% Os = (max_value - final_value) / final_value * 100;
% 
% Ess = input_deg - final_value;
% 
% % plot : step response (rate tracking)
% figure(3); clf;
% grid on; hold on;
% plot(time, step_out);
% plot(tr, final_value*0.9, 'r*', 'MarkerSize', 12);
% plot(time(os_idx), max_value, 'g*', 'MarkerSize', 12);
% xlabel('time [sec]'); ylabel('\omega [deg/s]');
% title('Step Response (rate tracking)');
% legend('', sprintf('rise time: %.4f [s]', tr), sprintf('overshoot: %.4f [%%]', Os));
% 
% str = {
%     sprintf('PM = %.1f [deg]', PM)
%     sprintf('GM = %.1f [dB]', GM_dB)
%     sprintf('Ess = %.2f [deg/s]', Ess)
%     sprintf('|e/wb|@wmax = %.3f', P_at_wmax)
% };
% annotation('textbox', 'String', str, 'FitBoxToText','on', 'BackgroundColor','w');

%disturbance transfer function magnitude  (high-pass : rejects low-freq w_b)
figure(4); clf;
bodemag(P, {1e-1, 1e3});
grid on; hold on;
title('Disturbance TF  P(s) = e / \omega_b   (high-pass)');

% figure; margin(Go);
% figure; step(Gcl);


% ---------------------------------------
%   Run Simulink model and get results
% ---------------------------------------
tstart = 0.0;
tend = 1.0;
fs = 200; % sampling freq
Ts = 1/fs ; % sampling period
dz    =28*(pi/180); 
W_sat = 1400;

model_name = 'Stabilization_simulink.slx';   % <-- 실제 .slx 파일 이름으로 변경

out = sim(model_name);

t   = out.t_out;        % 시간 벡터
y   = out.simout;       % 시뮬레이션 결과 데이터


time_meas= D_wLPF(:,1);
timeidx = find(time_meas >= 1.0 & time_meas <= 1.5);
omega_h = D_woLPF(:, 6);
omega_lpf = D_wLPF(:, 7);

% ---- 실험 결과 rise time/ overshoot 계산 ----
omega_mean = mean(omega_lpf(timeidx));
[~, tr_idx_exp] = min(abs(omega_lpf - omega_mean*0.9));
tr_exp = time_meas(tr_idx_exp);

[max_value_exp, os_idx_exp] = max(omega_lpf);
Os_exp = (max_value_exp - omega_mean) / omega_mean * 100 ;

% ---- 시뮬레이션 결과 rise time / overshoot 계산 ----
final_value_sim = y(end);
[~, tr_idx_sim] = min(abs(y - final_value_sim*0.9));
tr_sim = t(tr_idx_sim);

[max_value_sim, os_idx_sim] = max(y);
Os_sim = (max_value_sim - final_value_sim) / final_value_sim * 100;

Ess_sim = input_deg - final_value_sim;

figure(5);
grid on; hold on;
plot(time_meas, omega_lpf, 'r', 'linewidth', 1.3);
plot(time_meas, omega_h, 'g', 'LineWidth', 1.3);
plot(t, input_deg*ones(size(t)), 'k--', 'LineWidth', 1.2);
plot(t, y, 'b', 'linewidth', 1.3);
plot(tr_sim, final_value_sim*0.9, 'b*', 'MarkerSize', 12);  % simul tr
plot(t(os_idx_sim), max_value_sim, 'bo', 'MarkerSize', 12); % simul os
plot(tr_exp, omega_mean*0.9, 'r*', 'MarkerSize', 12);       % exp tr
plot(time_meas(os_idx_exp), max_value_exp, 'ro', 'MarkerSize', 12); % exp os
xlim([0, 0.7]);

xlabel('time [sec]'); ylabel('\omega [deg/s]');
legend('omega(lpf)', 'omega_h', 'omega(cmd)', 'Simulation', ...
    sprintf('rise time(simul) = %.4f [s]', tr_sim), ...
sprintf('overshoot(simul) = %.4f [%%]', Os_sim), ...
    sprintf('rise time(exp) = %.4f [s]', tr_exp), ...
    sprintf('overshoot(exp) = %.4f [%%]', Os_exp), 'location', 'southeast');
title(sprintf('Stabilization Performance (Simulation Vs Experiment) \\omega_c=%.f', Wc));

str_sim = {
    sprintf('Ess = %.2f [deg/s]', Ess_sim)
};
annotation('textbox', [0.15 0.15 0.1 0.05], 'String', str_sim, 'FitBoxToText','on', 'BackgroundColor','w');

