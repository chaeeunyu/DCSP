%% plot_bode_omega.m (한 주기 평균화 버전)
% 주파수별 raw_fX.XXHz.out 파일 50개에 대해
% 안정한 주기들을 평균내어 '딱 한 주기(1 Cycle)'만 subplot으로 플랏
clear; clc; close all;

%% ---- 설정 ---------------------------------------------------------------
data_dir  = 'bode_data';
n_header  = 3;
freq_list = 0.10 : 0.10 : 5.00;   % 50개 주파수
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
        figure('Name', sprintf('Averaged 1-Cycle: OmegaCmd vs Omega (Fig %d/%d)', fig_idx, n_figs), ...
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
    
    t         = data(:, 1);   % Time [s]
    omega_cmd = data(:, 2);   % OmegaCmd [deg/s]
    omega     = data(:, 6);   % Omega [rad/s]
    
    %% --- 💡 한 주기 평균화 알고리즘 (Cycle Averaging) -------------------
    T = 1 / freq;               % 해당 주파수의 한 주기 시간 [s]
    start_cycle = 2;            % 1번째 주기(과도응답)는 제외하고 2번째부터 사용
    end_cycle = floor(t(end)/T);% 파일에 포함된 마지막 온전한 주기 번호 (보통 4 또는 5)
    
    n_grid_points = 200;        % 한 주기를 쪼갤 공통 격자 점의 개수
    t_grid = linspace(0, T, n_grid_points)'; % 0 ~ T 까지의 표준 시간 축
    
    % 평균을 내기 위한 누적 버퍼 초기화
    cmd_sum   = zeros(n_grid_points, 1);
    omega_sum = zeros(n_grid_points, 1);
    cycle_count = 0;
    
    % 각 주기를 잘라서 공통 격자(t_grid)에 맞춰 더해줌 (보간법 적용)
    for c = start_cycle : end_cycle
        t_start = (c - 1) * T;
        t_end   = c * T;
        
        % 현재 주기 영역에 해당하는 데이터 인덱스 추출
        idx = (t >= t_start) & (t < t_end);
        
        if sum(idx) > 5  % 데이터 포인트가 유효하게 존재할 때만 처리
            t_cycle   = t(idx) - t_start;       % 0 ~ T 범위로 시간 오프셋 리셋
            cmd_cycle = omega_cmd(idx);         % [deg/s]
            omega_cycle = omega(idx) * (180/pi); % [rad/s] -> [deg/s] 변환
            
            % 샘플링 위치가 주기마다 미세하게 다를 수 있으므로 위상을 딱 맞추기 위해 보간법 사용
            cmd_interp   = interp1(t_cycle, cmd_cycle,   t_grid, 'linear', 'extrap');
            omega_interp = interp1(t_cycle, omega_cycle, t_grid, 'linear', 'extrap');
            
            cmd_sum     = cmd_sum + cmd_interp;
            omega_sum   = omega_sum + omega_interp;
            cycle_count = cycle_count + 1;
        end
    end
    
    % 정상적으로 주기가 누적되었다면 평균값 계산
    if cycle_count > 0
        cmd_avg   = cmd_sum / cycle_count;
        omega_avg = omega_sum / cycle_count;
    else
        warning('주기 분할 실패(데이터 부족): %s', fname);
        continue;
    end
    
    %% --- subplot 플랏 ----------------------------------------------------
    subplot(n_rows, n_cols, sub_idx);
    
    % 평균 데이터 기반으로 플랏 (X축은 0 ~ T 까지의 단일 주기 시간)
    plot(t_grid, cmd_avg,   'b-', 'LineWidth', 1.5, 'DisplayName', '\omega_{cmd}'); hold on;
    plot(t_grid, omega_avg, 'r-', 'LineWidth', 1.2, 'DisplayName', '\omega_{meas}');
    hold off;
    
    title(sprintf('f = %.2f Hz (Avg of %d cycles)', freq, cycle_count), 'FontSize', 10);
    xlabel('Time within a cycle [s]', 'FontSize', 8);
    ylabel('\omega [deg/s]',          'FontSize', 8);
    legend('Location', 'best', 'FontSize', 7);
    grid on;
    xlim([0 T]); % X축 범위를 정확히 한 주기(0 ~ T)로 고정
    
    % --- Figure 전체 제목 (4개 다 채웠거나 마지막 파일일 때) -------------
    if sub_idx == n_per_fig || k == n_freq
        freq_start = freq_list((fig_idx - 1) * n_per_fig + 1);
        freq_end   = freq_list(min(fig_idx * n_per_fig, n_freq));
        sgtitle(sprintf('Averaged 1-Cycle (Cmd vs Meas) — %.2f ~ %.2f Hz', ...
                        freq_start, freq_end), ...
                'FontSize', 12, 'FontWeight', 'bold');
    end
end

fprintf('완료! %d개 파일 → %d개 Figure 생성 (한 주기 평균화 완료).\n', n_freq, n_figs);