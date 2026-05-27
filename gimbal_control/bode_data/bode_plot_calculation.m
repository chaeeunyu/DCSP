% =========================================================
%  process_sweep.m
%  주파수 스윕 raw 파일들 → bode_result.out
% ---------------------------------------------------------
%  입력 : OmegaCmd[deg/s]  (2번째 컬럼) -> 코드 내에서 rad/s 변환
%  출력 : Omega[rad/s]     (6번째 컬럼)
%  방법 : 기본 주파수에서 Goertzel DFT → H = X_out / X_in
%  결과 : bode_result.out  [freq(Hz)  mag(dB)  phase(deg)]
% =========================================================
close all; clear; clc;

SKIP_CYCLES = 2;            % 앞부분 과도응답 제거 사이클 수
OUTPUT_FILE = 'bode_result.out';

% ── 파일 탐색 ─────────────────────────────────────────────
files = dir('raw_f*.out');
if isempty(files)
    error('raw_f*.out 파일을 찾을 수 없습니다. 같은 폴더에서 실행하세요.');
end

fprintf('파일 %d개 발견\n\n', numel(files));
fprintf('  %-30s %10s %10s %12s\n', '파일명', 'freq[Hz]', 'mag[dB]', 'phase[deg]');
fprintf('  %s\n', repmat('-', 1, 65));

results = zeros(numel(files), 3);   % [freq_hz, mag_dB, phase_deg]
valid   = false(numel(files), 1);

for k = 1 : numel(files)
    fpath = files(k).name;
    try
        % ── 헤더에서 주파수 파싱 ──────────────────────────
        fid   = fopen(fpath, 'r');
        line1 = fgetl(fid);
        fgetl(fid);   % line2 사용 안 함
        fclose(fid);
        
        tok = regexp(line1, 'f=([\d.]+)Hz', 'tokens');
        if isempty(tok)
            warning('주파수 파싱 실패: %s — 건너뜀', fpath);
            continue;
        end
        freq_hz = str2double(tok{1}{1});
        
        % ── 데이터 로드 (3줄 헤더 스킵) ──────────────────
        data = readmatrix(fpath, 'FileType', 'text', ...
                          'NumHeaderLines', 3);
        
        t         = data(:, 1);   % Time[s]
        omega_cmd = data(:, 2);   % OmegaCmd [deg/s] (2번째 컬럼) ← 변경된 입력
        omega     = data(:, 6);   % Omega [rad/s]     (6번째 컬럼) ← 출력
        
        dt = t(2) - t(1);
        
        % ── 💡 단위 통일 (중요!) ─────────────────────────
        % 출력이 rad/s이므로 입력(deg/s)도 rad/s로 맞춰야 
        % 시스템이 1:1로 완벽히 추종할 때 저주파 Gain이 0dB로 나옵니다.
        % 만약 raw 파일의 OmegaCmd가 이미 rad/s 단위라면 아래 줄을 주석 처리하세요.
        omega_cmd_rad = omega_cmd * (pi / 180); 
        
        % ── 과도응답 구간 제거 ────────────────────────────
        T_period     = 1.0 / freq_hz;
        skip_samples = round(SKIP_CYCLES * T_period / dt);
        
        t_ss    = t(skip_samples+1 : end);
        inp_ss  = omega_cmd_rad(skip_samples+1 : end);   % 단위 통일된 입력 사용
        out_ss  = omega(skip_samples+1 : end);
        
        % ── 기본 주파수에서 DFT (Goertzel-style) ─────────
        %   H(jw) = sum(out * e^{-jwt}) / sum(inp * e^{-jwt})
        omega_rad = 2 * pi * freq_hz;
        k_vec     = exp(-1j * omega_rad * t_ss);
        
        X_in  = dot(inp_ss,  k_vec);
        X_out = dot(out_ss,  k_vec);
        
        if abs(X_in) < 1e-12
            warning('입력 신호가 너무 작음: %s — 건너뜀', fpath);
            continue;
        end
        
        H = X_out / X_in;
        mag_dB    = 20 * log10(abs(H));
        phase_deg = angle(H) * 180 / pi;
        
        results(k, :) = [freq_hz, mag_dB, phase_deg];
        valid(k)      = true;
        
        fprintf('  %-30s %10.3f %10.4f %12.4f\n', ...
                fpath, freq_hz, mag_dB, phase_deg);
    catch ME
        warning('처리 오류 (%s): %s', fpath, ME.message);
    end
end

% ── 유효 결과만 추출 & 주파수 순 정렬 ────────────────────
results = results(valid, :);
results = sortrows(results, 1);

if isempty(results)
    error('처리된 파일이 없습니다.');
end

% ── bode_result.out 저장 ──────────────────────────────────
fid = fopen(OUTPUT_FILE, 'w');
fprintf(fid, '%-14s %-14s %-14s\n', 'freq[Hz]', 'mag[dB]', 'phase[deg]');
fprintf(fid, '%14.6f %14.6f %14.6f\n', results');
fclose(fid);

fprintf('\n✅ 저장 완료: %s  (%d개 주파수 포인트)\n', OUTPUT_FILE, size(results,1));