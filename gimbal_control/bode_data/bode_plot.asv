% ---------------------------------------------%
%         transfer function estimation         %
% ---------------------------------------------%
close all; clear all; clc;

data = readmatrix("bode_result.out", 'FileType', 'text', ...
                  'NumHeaderLines', 1);          % ← 헤더 1줄 스킵

tblOmega    = 2 * pi * data(:,1);               % Hz → rad/s
tblMagAtt   = data(:,2);                        % [dB]
tblMagLin   = 10.^(tblMagAtt / 20);            % ← dB → linear (추가)
tblPhsDelay = data(:,3) * pi / 180;             % deg → rad
tblFreqResp = tblMagLin .* exp(1j * tblPhsDelay);

wt= 1 ./(abs(tblOmega) * eps);

Nnum = 0;
Nden = 2;
[num, den] = invfreqs(tblFreqResp, tblOmega, Nnum, Nden, wt);
EstTF = tf(num, den);
figure; pzmap(EstTF); grid on;

figure; bode(EstTF); grid on;
figure;
subplot(2,1,1)
semilogx(tblOmega, tblMagAtt, 'bo', 'DisplayName', '측정값'); hold on;
[mag,phs,w] = bode(EstTF, tblOmega);
semilogx(w, 20*log10(squeeze(mag)), 'r-', 'DisplayName', '추정 TF');
ylabel('Magnitude [dB]'); legend; grid on;

subplot(2,1,2)
semilogx(tblOmega, data(:,3), 'bo'); hold on;
semilogx(w, squeeze(phs), 'r-');
ylabel('Phase [deg]'); grid on;