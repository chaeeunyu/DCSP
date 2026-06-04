% ---------------------------------------
%     Designation Loop Design
%  - Controller : PD
%  - Motor tf : 1st
% ---------------------------------------
clear; close all; clc;

% initialize
Km = 9.993 ;
Pm = 10.87;
W_sat = 1400;
dz    =28*(pi/180); 
dz_deg = 28;

% Design parameter --- PD-controller
Wc = 27.0;  % <------------------------------------------ MODIFY
Zc = 0.7;  % <------------------------------------------- MODIFY

Kp = Wc^2 / Km ;       % unit: [1/s]
Kd = (2*Zc*Wc - Pm) / Km ;  % unit: [-] dimensionless

% transfer function
s = tf('s');
Gm = Km / (s + Pm);    % input: omega_c [deg/s], output: omega [deg/s]
% Gc = Kp + Kd * s ;
Go_vel = Kp*Gm / (1 + Kd*Gm) * (1/s);
Go_vel = minreal(Go_vel);
Gcl = minreal(Go_vel / (1 + Go_vel));

% [GM, PM, Phase crossover freq, Gain Crossover freq]
[GM, PM, wpc, wgc] = margin(Go_vel);
GM_dB = 20*log10(GM);

% nyquist plot
figure(1); 
nyquist(Go_vel);
grid on; hold on;

% pz map
figure(2);
pzmap(Gcl);
grid on; hold on;

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


% plot
figure(3);
grid on; hold on;
plot(time, step_out);
plot(tr, final_value*0.9, 'r*', 'MarkerSize', 12);
plot(time(os_idx), max_value, 'g*', 'MarkerSize', 12);
xlabel('time [sec]'); ylabel('\psi [deg]');
title('Step Response');
legend('', sprintf('rise time: %.4f [s]', time(tr_idx)), sprintf('overshoot: %.4f [%%]', Os));

str = {
    sprintf('PM = %.1f [deg]', PM)
    sprintf('GM = %.1f [dB]', GM_dB)
    sprintf('Ess = %.2f [deg]', Ess)
};

annotation('textbox', 'String', str, 'FitBoxToText','on', 'BackgroundColor','w');

% figure; margin(Go_vel);
% figure; step(Gcl);
% grid on;
