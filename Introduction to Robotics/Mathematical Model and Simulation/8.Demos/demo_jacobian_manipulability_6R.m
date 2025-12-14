clear; clc; close all;

% Add toolbox folders to path
startup_MDH;   % or startup_MDH();

% Load the 6R MDH robot
[MDH, qmin, qmax] = getMDH_6R();

% Test configuration (rad)
q = [0.3; -0.6; 0.4; 0.2; -0.3; 0.1];

disp('Joint configuration q (rad):');
disp(q.');

% Jacobian
J = jacobianMDH(MDH, q);

% Manipulability metrics
[w, kappa] = manipulabilityMDH(MDH, q);

disp("Jacobian J (6 x n):");
disp(J);

fprintf("Yoshikawa manipulability w = %.6g\n", w);
fprintf("Jacobian condition number kappa = %.6g\n", kappa);
