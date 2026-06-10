% ---------------------------------------
%     Stabilization Loop Design
%  - Controller : PI
%  - Motor tf : 1st
% ---------------------------------------
clear; close all; clc;

% initialize
Km = 9.993 ;
Pm = 10.87;
W_sat = 1400;
dz    = 28*(pi/180);
dz_deg = 28;

% disturbance spec frequency (robot body bandwidth ~ 0.5 Hz)
Wmax = 2*pi*0.5;   % [rad/s]  <----------------------------- MODIFY (use given spec)

% Design parameter --- PI-controller
Wc = 24.0;  % <-------------------------------------------- MODIFY
Zc = 0.6;   % <-------------------------------------------- MODIFY

%  PD : Kp = Wc^2/Km ,        Kd = (2*Zc*Wc - Pm)/Km
%  PI : Ki = Wc^2/Km ,        Kp = (2*Zc*Wc - Pm)/Km   (dual structure)
Ki = Wc^2 / Km ;            % unit: [1/s]   (integral gain)
Kp = (2*Zc*Wc - Pm) / Km ;  % unit: [-]     (proportional gain)

% command magnitudes (read by the Simulink model from the base workspace)
input_deg = 5 ;     % [deg/s]  commanded rate w_c      <--- MODIFY
wb_dist   = 10 ;    % [deg/s]  body-rate disturbance w_b <-- MODIFY

% transfer function
s = tf('s');
Gm = Km / (s + Pm);     % input: omega_c [deg/s], output: omega [deg/s]
Gc = Kp + Ki/s ;        % PI controller

Go  = Gc * Gm ;                  % open loop (rate loop)
Gcl = minreal( Go / (1 + Go) );  % command tracking : w_c -> w_h
P   = minreal( -1 / (1 + Go) );  % disturbance tf  : w_b -> e   ( = e/w_b )

% [GM, PM, Phase crossover freq, Gain crossover freq]
[GM, PM, wpc, wgc] = margin(Go);
GM_dB = 20*log10(GM);

% disturbance-rejection spec : |e/w_b| at Wmax   (spec : <= 0.1)
P_at_wmax = abs( evalfr(P, 1j*Wmax) );

% nyquist plot
figure(1);
nyquist(Go);
grid on; hold on;

% pz map
figure(2);
pzmap(Gcl);
grid on; hold on;

% step input (rate command)
Tf = 0.5;
time = 0:0.001:Tf;
step_input = input_deg * ones(size(time));
step_out = lsim(Gcl, step_input, time);
final_value = step_out(end);
% rise time (90%)
[~, tr_idx] = min(abs(step_out - final_value*0.9));
tr = time(tr_idx);
% overshoot
[~, os_idx] = max(step_out);
max_value = step_out(os_idx);
Os = (max_value - final_value) / final_value * 100;

Ess = input_deg - final_value;

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
