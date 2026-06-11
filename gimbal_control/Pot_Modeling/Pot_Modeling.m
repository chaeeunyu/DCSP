%% ======================================================================
%  pot_angle_fit.m
%  ----------------------------------------------------------------------
%  pot_record_cw*.out / pot_record_ccw*.out 파일들을 읽어,
%  각 파일의 Pot[V] '평균값'을 구한 뒤
%   - x축 : 각도[deg]  (cw = +, ccw = -)
%   - y축 : Pot[V]
%  로 1차(직선) 피팅하여 기울기(slope)를 계산한다.
%
%  파일은 dir()로 자동 검색하므로, 나중에 cw/ccw 파일이
%  추가되면 코드 수정 없이 그대로 반영된다.
%  경로는 모두 상대 경로로 처리한다.
% =======================================================================

clear; clc; close all;

%% --- 설정 -------------------------------------------------------------
data_dir  = '.';                 % .out 파일이 있는 폴더 (상대 경로)
file_glob = 'pot_record_*.out';  % 검색 패턴
pot_col   = 3;                   % Pot[V] 컬럼 위치 (Time,Vg,Pot,Omega)
cw_sign   = +1;                  % cw  방향 각도 부호
ccw_sign  = -1;                  % ccw 방향 각도 부호 (부호 바꾸려면 여기 수정)

%% --- 파일 수집 --------------------------------------------------------
files = dir(fullfile(data_dir, file_glob));
if isempty(files)
    error('"%s" 폴더에서 "%s" 패턴 파일을 찾지 못했습니다.', data_dir, file_glob);
end

angle = [];     % 부호 있는 각도 [deg]
pot   = [];     % Pot 평균 전압 [V]
names = {};     % 파일명

for k = 1:numel(files)
    fname = files(k).name;

    % 파일명에서 방향(cw/ccw)과 각도 크기 파싱
    %   예) pot_record_ccw12.out -> dir='ccw', mag=12
    tok = regexp(fname, 'pot_record_(ccw|cw)(\d+)', 'tokens', 'once');
    if isempty(tok)
        warning('인식 못한 파일명, 건너뜀: %s', fname);
        continue;
    end
    dir_str = tok{1};
    mag     = str2double(tok{2});

    if strcmpi(dir_str, 'ccw')
        ang = ccw_sign * mag;
    else
        ang = cw_sign  * mag;
    end

    % Pot[V] 평균 계산
    v = read_pot_mean(fullfile(data_dir, fname), pot_col);

    angle(end+1,1) = ang;     %#ok<SAGROW>
    pot(end+1,1)   = v;       %#ok<SAGROW>
    names{end+1,1} = fname;   %#ok<SAGROW>
end

% 각도 순으로 정렬 (보기 좋게)
[angle, idx] = sort(angle);
pot   = pot(idx);
names = names(idx);

%% --- 1차 피팅 ---------------------------------------------------------
p         = polyfit(angle, pot, 1);   % p(1)=기울기, p(2)=절편
slope     = p(1);                     % [V/deg]
intercept = p(2);                     % [V]

% 결정계수 R^2
pot_fit = polyval(p, angle);
ss_res  = sum((pot - pot_fit).^2);
ss_tot  = sum((pot - mean(pot)).^2);
R2      = 1 - ss_res / ss_tot;

fprintf('\n=== 1차 피팅 결과 :  Pot[V] = slope*angle + intercept ===\n');
fprintf('  사용한 점 개수 : %d\n',          numel(angle));
fprintf('  기울기 slope   : %.6f V/deg\n',  slope);
fprintf('  절편 intercept : %.6f V\n',      intercept);
fprintf('  R^2            : %.6f\n',        R2);

%% --- 그래프 -----------------------------------------------------------
figure('Color','w');
plot(angle, pot, 'o', 'MarkerSize', 7, ...
     'MarkerFaceColor', [0.20 0.40 0.80], ...
     'MarkerEdgeColor', [0.10 0.20 0.50], ...
     'DisplayName', 'Measured (mean)');
hold on; grid on;

xfit = linspace(min(angle), max(angle), 100);
plot(xfit, polyval(p, xfit), 'r-', 'LineWidth', 1.5, ...
     'DisplayName', sprintf('Fit: %.5f V/deg (R^2=%.4f)', slope, R2));

xlabel('Angle [deg]');
ylabel('Pot [V]');
title('Potentiometer Voltage vs Angle');
legend('Location', 'best');

%% --- 결과 테이블 ------------------------------------------------------
T = table(names, angle, pot, 'VariableNames', {'File','Angle_deg','Pot_V'});
disp(T);

%% ======================================================================
%  로컬 함수 : 파일에서 Pot[V] 평균값 읽기
%  - '%' 주석줄, 빈줄, 컬럼명줄(Time[s]...) 등 숫자가 아닌 줄은 자동 무시
% =======================================================================
function v = read_pot_mean(filepath, pot_col)
    fid = fopen(filepath, 'r');
    if fid < 0
        error('파일을 열 수 없습니다: %s', filepath);
    end

    data = [];
    while ~feof(fid)
        line = fgetl(fid);
        if ~ischar(line); break; end
        nums = sscanf(line, '%f');          % 텍스트 줄이면 빈 결과
        if numel(nums) >= pot_col           % 정상 데이터 행만 채택
            data(end+1, 1:numel(nums)) = nums(:)';  
        end
    end
    fclose(fid);

    if isempty(data)
        error('숫자 데이터를 찾지 못했습니다: %s', filepath);
    end
    v = mean(data(:, pot_col));
end