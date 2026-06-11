% ---------------------------------------
%     Designation Loop Design
%  - Controller : PD
%  - Motor tf : 1st
% ---------------------------------------
clear; close all; clc;

% initialize
Km =  13.55;
Pm = 14.06 ;  % []
W_sat = 1400;
dz    =28*(pi/180); 
dz_deg = 28;

% Design parameter --- PD-controller
Wc = 26;  % [rad/s] <================================ MODIFY
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


% ---------------------------- nyquist plot ------------------------------------------
% [GM, PM, Phase crossover freq, Gain Crossover freq]
[GM, PM, wpc, wgc] = margin(Go_vel);
GM_dB = 20*log10(GM);

figure(1); 
[re, im, wout] = nyquist(Go_vel);
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
wpc_complex = evalfr(Go_vel, 1j*wpc);
wgc_complex = evalfr(Go_vel, 1j*wgc);
wpc_x = real(wpc_complex); wpc_y = imag(wpc_complex);
wgc_x = real(wgc_complex); wgc_y = imag(wgc_complex);
plot(wpc_x, wpc_y, '*', 'markeredgecolor', 'm', 'MarkerSize', 10, 'LineWidth', 1);
plot(wgc_x, wgc_y, '*', 'markeredgecolor', 'g', 'MarkerSize', 10, 'LineWidth', 1);
legend('', '', '', '', '', sprintf('GM = %.2f [dB] / wpc = %.2f [Hz]', GM_dB, wpc/(2*pi)), ...
    sprintf('PM = %.2f [deg] / wgc = %.2f [Hz]', PM, wgc/(2*pi)));
title('Nyquist Plot, Go_{vel}(s)');
xlabel('Re\{Go_{vel}(jw)\} [-]'); ylabel('Im\{Go_{vel}(jw)\} [-]');


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

% step input
Tf = 0.5;
time = 0:0.001:Tf;
input_deg = 5;     % [deg]   <--------------------------- MODIFY
step_input = input_deg * ones(size(time));
step_out = lsim(Gcl, step_input, time);
final_value = step_out(end);
% rise time
[~, tr_idx] = min(abs(step_out - final_value*0.9));
tr = time(tr_idx);
% overshoot
[~, os_idx] = max(step_out);
max_value = step_out(os_idx);
Os = (max_value - final_value) / final_value * 100;

Ess = input_deg - final_value;


% --------------------------------- step response -----------------------------------------
figure(3);
grid on; hold on;
yline(step_out(end), 'k--', 'LineWidth', 1.5);
plot(time, step_out, 'LineWidth', 1.5);
plot(tr, final_value*0.9, 'r*', 'MarkerSize', 12);
plot(time(os_idx), max_value, 'g*', 'MarkerSize', 12);
xlabel('time [sec]'); ylabel('\psi [deg]');
title('Step Response');
legend('', '', sprintf('rise time: %.4f [s]', time(tr_idx)), sprintf('overshoot: %.4f [%%]', Os));

str = {
    sprintf('PM = %.1f [deg]', PM)
    sprintf('GM = %.1f [dB]', GM_dB)
    sprintf('Ess = %.2f [deg]', Ess)
};

annotation('textbox', 'String', str, 'FitBoxToText','on', 'BackgroundColor','w');

% figure; margin(Go_vel);
% figure; step(Gcl);
% grid on;
