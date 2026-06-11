%% =========================================================
%  Slew Rate Analysis  —  OmegaCmd Step Response
%  슬루레이트 = 초기 과도구간에서 가장 가파른 순간 기울기
%  단위: Omega → deg/s  /  Slew Rate → deg/s²
% =========================================================
clear; clc; close all;

%% ── 0. 스크립트 위치를 작업 디렉토리로 자동 설정 ─────────────────────────
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end
cd(script_dir);
fprintf('Working directory: %s\n', script_dir);

%% ── 1. USER SETTINGS ─────────────────────────────────────────────────────
t_plot_max   = 1.0;    % [s]  오버레이 플롯 x축 범위
t_transient  = 0.5;    % [s]  슬루레이트 탐색 범위 (초기 과도구간 끝)
win_sec      = 0.05;   % [s]  슬라이딩 윈도우 폭 (기울기 피팅 구간)
smooth_span  = 5;      % movmean 윈도우 (1 = 스무딩 없음)
R2D          = 180/pi;

%% ── 2. 파일 목록 수집 ────────────────────────────────────────────────────
files = dir(fullfile(script_dir, 'step_*.out'));
if isempty(files)
    error('step_*.out 파일을 찾을 수 없습니다: %s', script_dir);
end
[~, sidx] = sort({files.name});
files = files(sidx);
N = numel(files);
fprintf('%d개 파일 발견.\n\n', N);

%% ── 3. 파일명에서 OmegaCmd 값 파싱 ──────────────────────────────────────
omega_cmd_vals = zeros(N,1);
for k = 1:N
    tok = regexp(files(k).name, 'OmegaCmd([+-]?\d+)deg', 'tokens', 'once');
    if ~isempty(tok)
        omega_cmd_vals(k) = str2double(tok{1});
    end
end

%% ── 4. 데이터 읽기 함수 (헤더 자동 탐지) ────────────────────────────────
function [t, omega_rad] = readOutFile(fpath)
    fid = fopen(fpath, 'r');
    skip = 0;
    while ~feof(fid)
        line = fgetl(fid);
        if ischar(line)
            trimmed = strtrim(line);
            if ~isempty(trimmed) && (trimmed(1)=='-' || trimmed(1)=='+' || ...
                                     (trimmed(1)>='0' && trimmed(1)<='9'))
                break;
            end
        end
        skip = skip + 1;
    end
    fclose(fid);

    fid = fopen(fpath, 'r');
    raw = textscan(fid, '%f %f %f %f %f %f %f', ...
                   'HeaderLines', skip, 'CollectOutput', true);
    fclose(fid);

    if isempty(raw) || isempty(raw{1})
        t = []; omega_rad = []; return;
    end
    data      = raw{1};
    t         = data(:,1);
    omega_rad = data(:,6);
end

%% ── 5. 컬러맵 ────────────────────────────────────────────────────────────
n_pos = max(sum(omega_cmd_vals > 0), 1);
n_neg = max(sum(omega_cmd_vals < 0), 1);
cmap_pos = cool(n_pos);
cmap_neg = autumn(n_neg);
c_pos_idx = 0; c_neg_idx = 0;

%% ── 6. Figure 1 초기화 ───────────────────────────────────────────────────
fig1 = figure('Name','Omega Step Response Overlay','NumberTitle','off');
fig1.Position = [50 50 1200 620];
hold on; grid on; box on;
xlabel('t [s]', 'FontSize', 13);
ylabel('\omega [deg/s]', 'FontSize', 13);
title('All Step Responses Overlaid', 'FontSize', 14);

%% ── 7. 결과 배열 사전 초기화 ─────────────────────────────────────────────
for k = 1:N
    results(k).filename       = files(k).name;
    results(k).OmegaCmd_degs  = omega_cmd_vals(k);
    results(k).SlewRate_degs2 = NaN;
    results(k).t_peak_start   = NaN;
    results(k).t_peak_end     = NaN;
end

%% ── 8. 메인 루프 ─────────────────────────────────────────────────────────
for k = 1:N
    fpath = fullfile(script_dir, files(k).name);
    [t, omega_rad] = readOutFile(fpath);

    if isempty(t) || numel(t) < 10
        warning('데이터 부족, 건너뜀: %s', files(k).name); continue;
    end

    t        = t - t(1);
    omega_deg = omega_rad * R2D;
    omega_sm  = movmean(omega_deg, smooth_span);

    % 색상·선 스타일
    omega_cmd = omega_cmd_vals(k);
    if omega_cmd >= 0
        c_pos_idx = c_pos_idx + 1;
        col = cmap_pos(min(c_pos_idx, n_pos), :);  ls = '-';
    else
        c_neg_idx = c_neg_idx + 1;
        col = cmap_neg(min(c_neg_idx, n_neg), :);  ls = '--';
    end

    % 응답 신호 플롯
    mask_plot = t <= t_plot_max;
    plot(t(mask_plot), omega_sm(mask_plot), ls, ...
         'Color', col, 'LineWidth', 1.2, ...
         'DisplayName', sprintf('%+d deg/s', omega_cmd));

    % ── 슬루레이트: 초기 과도구간 슬라이딩 윈도우 최대 기울기 ────────────
    % 탐색 범위: t = 0 ~ t_transient
    mask_trans = t <= t_transient;
    t_tr   = t(mask_trans);
    om_tr  = omega_sm(mask_trans);

    % 샘플 평균 간격으로 윈도우 포인트 수 계산
    dt_avg  = mean(diff(t_tr));
    win_pts = max(3, round(win_sec / dt_avg));

    best_slope = 0;
    best_i     = 1;

    % 슬라이딩 윈도우 선형 피팅
    for i = 1 : (numel(t_tr) - win_pts + 1)
        idx_w  = i : i + win_pts - 1;
        p_w    = polyfit(t_tr(idx_w), om_tr(idx_w), 1);
        % 부호 포함 절대값 기준으로 가장 가파른 기울기 선택
        if abs(p_w(1)) > abs(best_slope)
            best_slope = p_w(1);
            best_i     = i;
        end
    end

    % 최대 기울기 구간 인덱스
    idx_best  = best_i : best_i + win_pts - 1;
    t_fit     = linspace(t_tr(idx_best(1)), t_tr(idx_best(end)), 60);
    p_best    = polyfit(t_tr(idx_best), om_tr(idx_best), 1);
    o_fit     = polyval(p_best, t_fit);

    % 슬루레이트 직선: 굵은 마젠타
    plot(t_fit, o_fit, '-', ...
         'Color', [1.0 0.08 0.58], ...
         'LineWidth', 2.5, ...
         'DisplayName', sprintf('Slew %+4d: %+.1f deg/s²', omega_cmd, best_slope));

    results(k).SlewRate_degs2 = best_slope;
    results(k).t_peak_start   = t_tr(idx_best(1));
    results(k).t_peak_end     = t_tr(idx_best(end));
end

xlim([0 t_plot_max]);
legend('show', 'Location', 'eastoutside', 'FontSize', 7, 'NumColumns', 2);

%% ── 9. 콘솔 결과 출력 ────────────────────────────────────────────────────
fprintf('%-40s  %12s  %16s  %10s  %10s\n', ...
        'File', 'OmegaCmd', 'SlewRate[deg/s²]', 't_start[s]', 't_end[s]');
fprintf('%s\n', repmat('-', 1, 92));
for k = 1:N
    fprintf('%-40s  %+12.1f  %+16.3f  %10.4f  %10.4f\n', ...
        results(k).filename, results(k).OmegaCmd_degs, ...
        results(k).SlewRate_degs2, ...
        results(k).t_peak_start, results(k).t_peak_end);
end

%% ── 10. Figure 2: Slew Rate 요약 ─────────────────────────────────────────
omega_cmds_all = [results.OmegaCmd_degs];
slew_degs2_all = [results.SlewRate_degs2];
pos_mask = omega_cmds_all > 0;
neg_mask = omega_cmds_all < 0;

fig2 = figure('Name','Slew Rate Summary','NumberTitle','off');
fig2.Position = [200 100 800 460];
hold on; grid on; box on;
plot(omega_cmds_all(pos_mask), slew_degs2_all(pos_mask), 'bo-', ...
     'MarkerFaceColor','b','LineWidth',1.8,'MarkerSize',6,'DisplayName','+cmd');
plot(omega_cmds_all(neg_mask), slew_degs2_all(neg_mask), 'rs-', ...
     'MarkerFaceColor','r','LineWidth',1.8,'MarkerSize',6,'DisplayName','-cmd');
xlabel('OmegaCmd [deg/s]', 'FontSize', 12);
ylabel('Slew Rate [deg/s²]', 'FontSize', 12);
title('Slew Rate vs OmegaCmd', 'FontSize', 14);
legend('show', 'Location', 'northwest');

%% ── 11. CSV 저장 ─────────────────────────────────────────────────────────
csv_path = fullfile(script_dir, 'slewrate_results.csv');
fid = fopen(csv_path, 'w');
fprintf(fid, 'Filename,OmegaCmd_degs,SlewRate_degs2,t_peak_start_s,t_peak_end_s\n');
for k = 1:N
    fprintf(fid, '%s,%g,%g,%g,%g\n', ...
        results(k).filename, results(k).OmegaCmd_degs, ...
        results(k).SlewRate_degs2, ...
        results(k).t_peak_start, results(k).t_peak_end);
end
fclose(fid);
fprintf('\nCSV 저장 완료: %s\n', csv_path);
