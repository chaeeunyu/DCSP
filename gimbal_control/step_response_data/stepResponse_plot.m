clear; close all; clc;

data = readmatrix("step_Vcmd+2.0000.out", 'FileType', 'text', 'NumHeaderLines', 5) ;

time = data(:, 1);
omega = data(:, 6) ;
%omega_target = data(:, 7) ;

num = [60.75];
den = [1 12.44];

t = 0:0.001:5.0 ;
EstTF = tf(num, den);
response = 2*step(EstTF, t);

figure;
grid on; hold on;
plot(time, omega, 'b', 'LineWidth', 1.5);
plot(t, response, 'r--', 'LineWidth', 1.5);
xlabel('time [sec]'); ylabel('omega[rad/sec]');
legend('omega', 'simulation');