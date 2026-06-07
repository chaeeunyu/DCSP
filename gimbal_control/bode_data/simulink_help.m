clear; close all; clc;

input_deg = pis_cmd(1);

Tstart = 0.0;
Tfinal = 5.0;
Tsim   = 0.001;

SimData = sim('filename');

SimData = struct();
SimData.tout % simulation time
SimData.psi  % user output
SimData.err1 % psi error

plot(SimData.tout, SimData.psi)
plot(time, psi);
