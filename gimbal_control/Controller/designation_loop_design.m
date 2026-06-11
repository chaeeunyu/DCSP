clear; clc;
%% ---- 1) 파라미터 (여기만 수정) --------------------------------
Km = 9.993;  Pm = 10.87;
zeta = 0.7;  wn = 27;
Kp = wn^2/Km;                 % = 72.95
Kd = (2*zeta*wn - Pm)/Km;     % = 2.695
w_sat = 1400*(pi/180);     % [rad/s] 측정한 포화(슬루율) 한계  <- 실제 측정값으로 교체
dz    =0*(pi/180);    % [rad/s] 측정한 데드존 반폭        <- 실제 측정값으로 교체
assignin('base','Kp',Kp);   assignin('base','Kd',Kd);
assignin('base','w_sat',w_sat); assignin('base','dz',dz);
fprintf('Kp=%.4f Kd=%.4f | w_sat=%.3f dz=%.3f [rad/s]\n', Kp,Kd,w_sat,dz);
%% ---- 2) 새 모델 -----------------------------------------------
m = 'designation_loop';
if bdIsLoaded(m), close_system(m,0); end
new_system(m);
add = @(src,name,pos,varargin) add_block(src,[m '/' name], ...
'Position',pos,varargin{:});
%% ---- 3) 블록 배치 ---------------------------------------------
add('simulink/Sources/Step','theta_d',[25 103 55 127], ...
'Time','0','Before','0','After','1');            % 계단(rad). 도단위면 15
add('simulink/Math Operations/Sum','Sum1',[95 104 115 124],'Inputs','+-');
add('simulink/Math Operations/Gain','Kp',[145 98 185 122],'Gain','Kp');
add('simulink/Math Operations/Sum','Sum2',[215 104 235 124],'Inputs','+-');
add('simulink/Discontinuities/Dead Zone','DeadZone',[260 97 300 123], ...
'LowerValue','-dz','UpperValue','dz');           % ★ 명령(입력) 데드존
add('simulink/Continuous/Transfer Fcn','Motor',[330 92 410 126], ...
'Numerator','[9.993]','Denominator','[1 10.87]');% -> omega(ideal)
add('simulink/Discontinuities/Saturation','Sat',[440 97 480 123], ...
'UpperLimit','w_sat','LowerLimit','-w_sat');     % omega 슬루율 한계
add('simulink/Continuous/Integrator','Integ',[510 95 540 125]); % omega->theta
add('simulink/Math Operations/Gain','Kd',[430 190 470 220], ...
'Gain','Kd','Orientation','left');               % rate gyro 피드백
add('simulink/Signal Routing/Mux','Mux',[585 97 590 143],'Inputs','2');
add('simulink/Sinks/Scope','Scope',[630 105 660 135]);          % theta_d, theta
add('simulink/Sinks/Scope','Scope_w',[630 190 660 220]);        % omega(실제)
%% ---- 4) 배선 --------------------------------------------------
L = @(a,b) add_line(m,a,b,'autorouting','on');
L('theta_d/1','Sum1/1');
L('Sum1/1','Kp/1');
L('Kp/1','Sum2/1');
L('Sum2/1','DeadZone/1');        % ★ 명령 -> 데드존 (입력단으로 이동)
L('DeadZone/1','Motor/1');       % ★ 데드존 -> 모터
L('Motor/1','Sat/1');            % 모터 -> 슬루율 포화 = 실제 omega
L('Sat/1','Integ/1');            % 실제 omega -> 적분 -> theta
L('Sat/1','Kd/1');               % 실제 omega -> rate gyro (포화 뒤 측정)
L('Sat/1','Scope_w/1');          % omega 파형 관찰 (슬루/데드존 확인용)
L('Kd/1','Sum2/2');              % 안쪽 합산점(-)
L('Integ/1','Sum1/2');           % 위치 피드백(-)
L('theta_d/1','Mux/1');
L('Integ/1','Mux/2');
L('Mux/1','Scope/1');
%% ---- 5) 솔버/시간 & 저장 --------------------------------------
set_param(m,'StopTime','0.45','Solver','ode45');
save_system(m);
open_system(m);
fprintf('생성 완료: %s.slx\n', m);
