clear; close all; clc;

D = readmatrix('tri_A400deg_F0.0250.out', ...
               'FileType','text', 'NumHeaderLines', 2);
t      = D(:,1);  Omega_cmd = D(:,2);
 om   = D(:,6);
 om=om*(180/pi);

figure;
subplot(2,1,1);
plot(t, Omega_cmd, 'b'); ylabel('Omega_cmd [deg/s]'); grid on;
title('dynamic validation');

subplot(2,1,2); hold on;
plot(t, Omega_cmd, 'k--', 'DisplayName', 'omega_input');
plot(t, om,     'r',   'DisplayName', 'Measured \omega');
ylabel('\omega [deg/s]'); xlabel('Time [s]');
legend; grid on;