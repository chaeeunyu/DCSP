clear all, close all, clc;

data = readmatrix('4_C.csv');

f = data(:,4);      % frequency
mag = data(:,5);    % magnitude

% 10Hz 근처 인덱스 찾기
[~, idx10] = min(abs(f - 10));
[~, idx105] = min(abs(f - 105));
[~, idx95] = min(abs(f - 95));

% 값 출력
% fprintf('10 Hz magnitude: %f\n', mag(idx10));
% fprintf('105 Hz magnitude: %f\n', mag(idx105));
% fprintf('95 Hz magnitude: %f\n', mag(idx95));

figure;
plot(f, mag);
grid on;
xlabel('Frequency [Hz]');
ylabel('Magnitude [dB]');
title('FFT at node 4 with capacitor')
hold on;

% 10Hz 표시
plot(f(idx10), mag(idx10), 'ro', 'MarkerSize', 8);
text(f(idx10)-10, mag(idx10)+3, ' 10Hz');

% 105Hz 표시
plot(f(idx105), mag(idx105), 'go', 'MarkerSize', 8);
text(f(idx105), mag(idx105), ' 105Hz');

% 95Hz 표시
plot(f(idx95), mag(idx95), 'yo', 'MarkerSize', 8);
text(f(idx95)-10, mag(idx95)+3, ' 95Hz');