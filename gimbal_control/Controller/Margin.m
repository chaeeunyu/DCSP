clear; clc;

Km=9.993; Pm=10.87; Kp=72.95; Kd=2.695;
L=tf(Km*Kp, [1, Pm+Km*Kd, 0]);
margin(L); grid on
[GM, PM, Wcg, Wcp] = margin(L);
fprintf("PM=%.1f deg, GM=%.2f dB\n", PM, 20*log10(GM));