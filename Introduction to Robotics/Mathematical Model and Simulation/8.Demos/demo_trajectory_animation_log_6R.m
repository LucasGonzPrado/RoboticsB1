clear; clc; close all;

% Add toolbox folders to MATLAB path
startup_MDH;   % or startup_MDH();

% Load robot
[MDH, qmin, qmax] = getMDH_6R();

% Joint-space start/end (rad)
q0 = [0; 0; 0; 0; 0; 0];
qf = [0.6; -0.8; 0.4; 0.5; -0.4; 0.2];

% Timing
tf = 4;
dt = 0.05;

% Quintic trajectory
[t, Q, Qd, Qdd] = jointTrajQuinticMDH(q0, qf, tf, dt);

% Animate
animateRobotTrajMDH(MDH, Q, dt, false);

% Animate with manipulability coloring
animateRobotTrajManipMDH(MDH, Q, dt);

% Log + dashboard plots
trajLog = logTrajMDH(MDH, t, Q, Qd, Qdd);
plotLogMDH(trajLog);
