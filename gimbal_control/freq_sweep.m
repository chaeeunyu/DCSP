%% plot_bode_omega.m
% 주파수별 raw_fX.XXHz.out 파일 50개에 대해
% Omega와 Omega_target을 각 Figure에 4개씩 subplot으로 플랏
%
% 파일 구조 (헤더 3줄 skip):
%   Col 1: Time[s]
%   Col 2: Vcmd[V]
%   Col 3: Vc[V]
%   Col 4: Vg[V]
%   Col 5: Pot[V]
%   Col 6: Omega[rad/s]
%   Col 7: Omega_target[rad/s]
clear; clc; close all;

%% ---- 설정 ---------------------------------------------------------------
data_dir  = 'bode_data';
n_header  = 3;
freq_list = 0.10 : 0.10 : 5.00;   % 50개
n_freq    = length(freq_list);

%% ---- Figure당 subplot 배치 설정 -----------------------------------------
n_per_fig = 4;          % Figure당 플랏 수
n_rows    = 2;
n_cols    = 2;
n_figs    = ceil(n_freq / n_per_fig);   % 필요한 Figure 수 (= 13)

%% ---- 데이터 읽기 & 플랏 -------------------------------------------------
for k = 1 : n_freq

    freq  = freq_list(k);
    fname = sprintf('raw_f%.2fHz.out', freq);
    fpath = fullfile(data_dir, fname);

    % --- 새 Figure 시작 (4개마다) ----------------------------------------
    fig_idx = ceil(k / n_per_fig);          % 현재 Figure 번호
    sub_idx = mod(k - 1, n_per_fig) + 1;   % Figure 내 subplot 위치 (1~4)

    if sub_idx == 1
        figure('Name', sprintf('Omega vs Omega\\_target  (Fig %d/%d)', fig_idx, n_figs), ...
               'NumberTitle', 'off', ...
               'Position', [50 50 900 700]);
    end

    % --- 파일 존재 여부 확인 ---------------------------------------------
    if ~isfile(fpath)
        warning('파일을 찾을 수 없습니다: %s', fpath);
        continue;
    end

    % --- 데이터 읽기 -----------------------------------------------------
    data = readmatrix(fpath, ...
                      'NumHeaderLines', n_header, ...
                      'FileType', 'text');

    t            = data(:, 1);
    omega        = data(:, 6);
    omega_target = data(:, 7);

    % --- subplot 플랏 ----------------------------------------------------
    subplot(n_rows, n_cols, sub_idx);
    plot(t, omega_target * 180/pi, 'b-', 'LineWidth', 1.2, 'DisplayName', '\omega_{target}'); hold on;
    plot(t, omega        * 180/pi, 'r-', 'LineWidth', 1.0, 'DisplayName', '\omega');
    hold off;

    title(sprintf('f = %.2f Hz', freq), 'FontSize', 10);
    xlabel('Time [s]',        'FontSize', 8);
    ylabel('\omega [deg/s]',  'FontSize', 8);
    legend('Location', 'best', 'FontSize', 7);
    grid on;
    xlim([t(1) t(end)]);

    % --- Figure 제목 (4개 다 채웠거나 마지막 파일일 때) -----------------
    if sub_idx == n_per_fig || k == n_freq
        freq_start = freq_list((fig_idx - 1) * n_per_fig + 1);
        freq_end   = freq_list(min(fig_idx * n_per_fig, n_freq));
        sgtitle(sprintf('Omega vs Omega\\_target  —  %.2f ~ %.2f Hz', ...
                        freq_start, freq_end), ...
                'FontSize', 12, 'FontWeight', 'bold');
    end
end

fprintf('완료! %d개 파일 → %d개 Figure 생성.\n', n_freq, n_figs);