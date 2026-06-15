% ---------------------------------------
%     Stabilization Loop Design
%  - Controller : PI
%  - Motor tf : 1st
% ---------------------------------------
clear; close all; clc;

% initialize
Km = 13.55 ;
Pm = 14.06;
W_sat = 1400;
dz    = 28*(pi/180);
dz_deg = 28;

% disturbance spec frequency (robot body bandwidth ~ 0.5 Hz)
Wmax = 2*pi*0.5;   % [rad/s]  <----------------------------- MODIFY (use given spec)

% Design parameter --- PI-controller
Wc = 20.0;  % <-------------------------------------------- MODIFY
Zc = 0.7;   % <-------------------------------------------- MODIFY

%  PD : Kp = Wc^2/Km ,        Kd = (2*Zc*Wc - Pm)/Km
%  PI : Ki = Wc^2/Km ,        Kp = (2*Zc*Wc - Pm)/Km   (dual structure)
Ki = Wc^2 / Km ;            % unit: [1/s]   (integral gain)
Kp = (2*Zc*Wc - Pm) / Km ;  % unit: [-]     (proportional gain)

% command magnitudes (read by the Simulink model from the base workspace)
omega_cmd = 0.0 ;     % [deg/s]  commanded rate w_c      <--- MODIFY
wb_dist   = 10 ;    % [deg/s]  body-rate disturbance w_b <-- MODIFY

% transfer function
s = tf('s');
Gm = Km / (s + Pm);     % input: omega_c [deg/s], output: omega [deg/s]
Gc = Kp + Ki/s ;        % PI controller

Go  = Gc * Gm ;                  % open loop (rate loop)
Gcl = minreal( Go / (1 + Go) );  % command tracking : w_c -> w_h
P   = minreal( -1 / (1 + Go) );  % disturbance tf  : w_b -> e   ( = e/w_b )

% disturbance-rejection spec : |e/w_b| at Wmax   (spec : <= 0.1)
P_at_wmax = abs( evalfr(P, 1j*Wmax) );


% ---------------------------- nyquist plot ------------------------------------------
% [GM, PM, Phase crossover freq, Gain Crossover freq]
[GM, PM, wpc, wgc] = margin(Go);
GM_dB = 20*log10(GM);

figure(1); 
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
figure(2);
pzmap(Gm, Gcl);
grid on; hold on;
legend(sprintf('Gm: p=%.2f±%.2f[Hz], z=%.2f[Hz]', real_pGm/(2*pi), imag_pGm/(2*pi), z_Gm), ...
    sprintf('Gcl: p=%.2f±%.2f[Hz], z=%.2f[Hz]\n', real_pGcl/(2*pi), imag_pGcl/(2*pi), z_Gcl));
% ---------------------------------------------------------------------------------

% step input (rate command)
Tf = 0.5;
time = 0:0.001:Tf;
step_input = omega_cmd * ones(size(time));
step_out = lsim(Go, step_input, time);
finsal_value = step_out(end);
% rise time (90%)
[~, tr_idx] = min(abs(step_out - final_value*0.9));
tr = time(tr_idx);
% overshoot
[~, os_idx] = max(step_out);
max_value = step_out(os_idx);
Os = (max_value - final_value) / final_value * 100;

Ess = omega_cmd - final_value;

% plot : step response (rate tracking)
figure(3);
grid on; hold on;
plot(time, step_out);
plot(tr, final_value*0.9, 'r*', 'MarkerSize', 12);
plot(time(os_idx), max_value, 'g*', 'MarkerSize', 12);
xlabel('time [sec]'); ylabel('\omega [deg/s]');
title('Step Response (rate tracking)');
legend('', sprintf('rise time: %.4f [s]', tr), sprintf('overshoot: %.4f [%%]', Os));

str = {
    sprintf('PM = %.1f [deg]', PM)
    sprintf('GM = %.1f [dB]', GM_dB)
    sprintf('Ess = %.2f [deg/s]', Ess)
    sprintf('|e/wb|@wmax = %.3f', P_at_wmax)
};
annotation('textbox', 'String', str, 'FitBoxToText','on', 'BackgroundColor','w');

% disturbance transfer function magnitude  (high-pass : rejects low-freq w_b)
figure(4);
bodemag(P, {1e-1, 1e3});
grid on; hold on;
title('Disturbance TF  P(s) = e / \omega_b   (high-pass)');

% figure; margin(Go);
% figure; step(Gcl);
