% ──────────────────────────────────────────────
%  Designation Loop : Tracking Performance Plot
%  validation
% ──────────────────────────────────────────────

d = readmatrix('dsg_psi+60deg_Wc=35.0, Zc=0.7_20260621_131417.out', 'Filetype', 'text', 'NumHeaderLines', 5);
t        = d(:,1);   % [s]
omega_c  = d(:,2);   % [deg/s]
Vc       = d(:,3);   % [V]
omega    = d(:,6);   % [rad/s]
psi      = d(:,7);   % [deg]

% ---- motor parameter -----
Km =  13.55;
Pm = 14.06 ;  % []

% ----- controller parameter ---------------
psi_cmd  = 60.0;      % [deg]  <------------------- MODIFY!!
Wc = 32;     %   <--------------------------------- MODIFY!!
Zc = 0.7;
Kp = Wc^2 / Km ;       % unit: [rad/s]
Kd = (2*Zc*Wc - Pm) / Km ;  % unit: [-] dimensionless

% transfer function
s = tf('s');
Gm = Km / (s + Pm);    % input: omega_c [deg/s], output: omega [deg/s]
% Gc = Kp + Kd * s ;
Go_vel = Kp*Gm / (1 + Kd*Gm) * (1/s);
Go_vel = minreal(Go_vel);
Gcl = minreal(Go_vel / (1 + Go_vel));


% ------------ rise time --------------
[~, idx] = min(abs(t - 1.0));
final_value   = mean(psi(idx:end));
initial_value = psi(1);

target90 = initial_value + 0.9 * (final_value - initial_value);
tr_idx   = find(psi >= target90, 1, 'first');
tr       = t(tr_idx);

Ess = psi_cmd - final_value;

% ----------- simulink plot -------------------
tstart = 0.0;
tend = 0.7;
fs = 200; % sampling freq
Ts = 1/fs ; % sampling period
dz    =28*(pi/180); 
W_sat = 1400;

model_name = 'Designation_simulink.slx';
out = sim(model_name);

simul_time = out.t_out;
simul_out = out.simout;

[~, sim_idx] = min(abs(simul_time - 1.0));
sim_final_value = mean(simul_out(sim_idx:end));
sim_init_value = simul_out(1);
sim_target90 = sim_init_value + 0.9 * (sim_final_value - sim_init_value);
sim_tr_idx   = find(simul_out >= sim_target90, 1, 'first');
sim_tr       = t(sim_tr_idx);

simEss = psi_cmd - final_value;

% ---------- plot ------------------------------
figure(1); clf;
hold on; grid on;
plot(t, psi,                   'b',   'LineWidth', 1.5, 'DisplayName', '\psi (actual)');
plot(t, psi_cmd*ones(size(t)), 'k--', 'LineWidth', 1.2, 'DisplayName', '\psi_{cmd}');
plot(simul_time, simul_out, 'r', 'LineWidth', 1.5, 'DisplayName', 'simul_{out}');
xlabel('Time [s]');
ylabel('\psi [deg]');
title(sprintf('<Position Tracking - input deg: %.f[deg]>', psi_cmd));
xlim([0, 0.7]);


% --------------------- risetime plot --------------------------
plot(t(tr_idx), target90, '*', 'color', '#4DBEEE', 'MarkerSize', 12, 'linewidth', 1.4, 'DisplayName', 'rise time pt');
plot(simul_time(sim_tr_idx), sim_target90, 'm*', 'MarkerSize', 12, 'linewidth', 1.4, 'DisplayName', 'rise time simul')

legend('Location','southeast');

str = { sprintf('Zc = %.1f [-]', Zc)
        sprintf('Wc = %.f [rad/s]', Wc)
        sprintf('Kp = %.3f [V/deg]', Kp)
        sprintf('Kd = %.4f [V*s/deg]', Kd)
        sprintf('--------------------')
        sprintf('<actual data>')
        sprintf('rise time = %.4f [s]', tr)
         sprintf('Ess = %.4f [deg]', Ess)
        };
        
annotation('textbox', [0.67 0.65 1 0.08], ...
'String',          str, ...
'FitBoxToText',    'on', ...
'BackgroundColor', 'w', ...
'FontSize',        10);

str2 = { sprintf('<simulation data>')
         sprintf('rise time = %.4f [s]', sim_tr)
         sprintf('Ess = %.4f [deg]', simEss) };

annotation('textbox', [0.4 0.43 0.5 0.1], ...
    'String',          str2, ...
    'FitBoxToText',    'on', ...
    'BackgroundColor', 'w', ...
    'FontSize',        10);


% ---------------------------- nyquist plot ------------------------------------------
% [GM, PM, Phase crossover freq, Gain Crossover freq]
[GM, PM, wpc, wgc] = margin(Go_vel);
GM_dB = 20*log10(GM);

figure(2); clf;
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
title(sprintf('Nyquist Plot, Go_{vel}(s)   (Wc=%.f, Zc=%.1f)', Wc, Zc));
xlabel('Re\{Go_{vel}(jw)\} [-]'); ylabel('Im\{Go_{vel}(jw)\} [-]');