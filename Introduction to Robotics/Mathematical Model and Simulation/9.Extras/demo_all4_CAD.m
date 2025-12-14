clear; clc; close all;
startup_MDH;

data = robotData_6R_CAD();
MDH  = data.MDH;

T_fixed.after = 1;
T_fixed.T     = data.T_12_fixed;

q_test = zeros(6,1);

% 1) Classification
info = singularityInfo6R_CAD(MDH, q_test, T_fixed);
disp(info.type);
disp([info.sigma_min, info.kappa]);

% 2) IK with avoidance
T_target = eye(4);
T_target(1:3,4) = [0.15; 0.00; 0.18];  % choose reachable
opts = struct('alphaNS',0.07,'kappaHigh',250,'maxIter',300);
[q_sol, ok, dbg] = ikineMDH_CAD_avoid(MDH, q_test, T_target, T_fixed, opts);
fprintf("IK ok = %d\n", ok);

figure; plot(dbg.err); grid on; title('IK error');

% 3) Singularity map
singularityMapWorkspaceMDH_CAD(MDH, data.qmin, data.qmax, T_fixed, 5000, 'sigma');

% 4) Your existing trajectory + animation can remain the same,
% as long as your plot/animate functions call fkineMDH_all_CAD internally.
