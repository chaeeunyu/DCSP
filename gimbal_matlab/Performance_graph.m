% ---------------------------------------------
%     Designation Loop Performance graph
%     - Controller : Velocity Feedback (Inexact D 대체)
% ---------------------------------------------
clear; close all; clc;

% 1. System Parameters
Km = 13.55; % <------------------- MODIFY!!!
Pm = 14.06; % <------------------- MODIFY!!!

% 2. Design Parameter Vectors
Wc_vec = 10:0.5:50;
Zc_vec = [0.5, 0.6, 0.7, 0.8, 0.9];

% transfer function
s = tf('s');
Gm = Km / (s + Pm);    % input: omega_c [rad/s], output: omega [rad/s]

tr_matrix  = zeros(length(Zc_vec), length(Wc_vec));
OS_matrix  = zeros(length(Zc_vec), length(Wc_vec));
Ess_matrix = zeros(length(Zc_vec), length(Wc_vec));
GM_matrix = zeros(length(Zc_vec), length(Wc_vec));
PM_matrix = zeros(length(Zc_vec), length(Wc_vec));
wpc_matrix = zeros(length(Zc_vec), length(Wc_vec));
wgc_matrix = zeros(length(Zc_vec), length(Wc_vec));


for i = 1:length(Zc_vec)
    Zc = Zc_vec(i);

    for j = 1:length(Wc_vec)
        Wc = Wc_vec(j);

        % Controller Gain
        Kp = Wc^2 / Km;
        Kd = (2*Zc*Wc - Pm) / Km;

        % tf
        Go_vel = (Kp * Gm) / (1 + Kd * Gm) * (1/s);
        Go_vel = minreal(Go_vel);
        Gcl    = minreal(Go_vel / (1 + Go_vel));

        % Steady-State Error - ramp input
        Ess_matrix(i,j) = (2 * Zc) / Wc;

        % tr, Os
        info = stepinfo(Gcl, 'RiseTimeLimits', [0 0.9]);
        tr_matrix(i,j) = info.RiseTime;
        OS_matrix(i,j) = info.Overshoot;

        % PM, GM
        [GM_matrix(i,j), PM_matrix(i,j), wpc_matrix(i,j), wgc_matrix(i,j)] = margin(Go_vel);
        GM_matrix(i,j) = 20*log10(GM_matrix(i,j));

    end

    wc_norm = Wc_vec/Pm;
    
    figure(1); % rise time
    hold on; grid on;
    plot(Wc_vec/Pm, tr_matrix(i,:), 'LineWidth', 2, 'DisplayName', sprintf('zeta_c = %.1f', Zc));

    figure(2); % Overshoot
    grid on; hold on;
    plot(Wc_vec/Pm, OS_matrix(i,:), 'LineWidth', 2, 'DisplayName', sprintf('zeta_c = %.1f', Zc));

    figure(3);  % steady-state error
    grid on; hold on;
    plot(Wc_vec/Pm, Ess_matrix(i,:), 'LineWidth', 2, 'DisplayName', sprintf('zeta_c = %.1f', Zc));

    figure(4);  % PM
    grid on; hold on;
    plot(Wc_vec/Pm, PM_matrix(i,:), 'LineWidth', 2, 'DisplayName', ...
        sprintf('zeta_c = %.1f / wgc = %.2f[Hz]', Zc, wgc_matrix(i,j)/(2*pi)));

    figure(5);  % GM
    grid on; hold on;
    plot(Wc_vec/Pm, GM_matrix(i,:), 'LineWidth', 2, 'DisplayName', ...
        sprintf('zeta_c = %.1f / wpc = %.2f[Hz]', Zc, wpc_matrix(i,j)/(2*pi)));
    
end

figure(1);
title('rise time 성능지표');
xlabel('Control Bandwidth, \omega_c/Pm [-]');  ylabel('Rising Time, t_{r}^{90} [sec]');
yline(0.1, 'r--', 'Target Spec (t_r \leq 0.1s)', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
legend('show', 'Location', 'northeast');

figure(2);
title('Overshoot 성능지표');
xlabel('Control Bandwidth, \omega_c/Pm [-]');  ylabel('Overshoot [%]');
yline(10, 'r--', 'Target Spec (%OS < 10%)', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
legend('show', 'Location', 'northeast');

figure(3);
title('Steady-state error 성능지표');
xlabel('Control Bandwidth, \omega_c/Pm [-]');  ylabel('Ess [deg]');
%yline(5, 'r--', 'Target Spec (Ess \leq 5[deg])', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
legend('show', 'Location', 'northeast');

figure(4);
title('PM 성능지표');
xlabel('Control Bandwidth, \omega_c/Pm [-]');  ylabel('PM [deg]');
yline(45, 'r--', 'Target Spec (PM \geq 45[deg])', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
legend('show', 'Location', 'northeast');

figure(5);
title('GM 성능지표');
xlabel('Control Bandwidth, \omega_c/Pm [-]');  ylabel('GM [dB]');
yline(10, 'r--', 'Target Spec (GM \geq 10[dB])', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
legend('show', 'Location', 'northeast');