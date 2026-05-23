data = readmatrix('% f=0.10Hz  n_cycles=5  N_total=100.txt', 'FileType', 'text', 'CommentStyle', '%', 'NumHeaderLines', 1);
 
Time         = data(:,1);
Vcmd         = data(:,2);
Omega        = data(:,6);
Omega_target = data(:,7);
 
figure;
 
subplot(2,1,1);
plot(Time, Vcmd, 'b');
ylabel('V_{cmd} [V]'); grid on;
 
subplot(2,1,2);
plot(Time, Omega, 'b'); hold on;
plot(Time, Omega_target, 'r--');
legend('\Omega', '\Omega_{target}');
xlabel('Time [s]'); ylabel('\Omega [rad/s]'); grid on;
 