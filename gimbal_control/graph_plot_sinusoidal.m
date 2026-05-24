clear; close all; clc;

D = readmatrix('sine_A2.00_F0.0250.out', ...
               'FileType','text', 'NumHeaderLines', 2);
t      = D(:,1);  vcmd = D(:,2);
om_tgt = D(:,7);  om   = D(:,6);

figure;
subplot(2,1,1);
plot(t, vcmd, 'b'); ylabel('Vcmd_{ref} [V]'); grid on;
title('dynamic validation (Vcmd=2[V], f=0.025[Hz])');

%subplot(2,1,2); 
figure(2);
hold on;
plot(t, om_tgt*180/pi, 'k--', 'DisplayName', 'Target \omega');
plot(t, om*180/pi,     'r',   'DisplayName', 'Measured \omega');
ylabel('\omega [deg/s]'); xlabel('Time [s]');
legend; grid on;