%% demo_master_CADrobot.m
% ---------------------------------------------------------
% Master demo for CAD-derived 6R robot (MDH + fixed offset)
%
% Demonstrates:
%   - Correct CAD geometry visualization
%   - FK, Jacobian, manipulability
%   - Workspace
%   - Joint-space trajectory
%   - Animation (normal + manipulability)
%   - Logging and plots
%   - Static + (gravity) torque along trajectory for a payload
% ---------------------------------------------------------

clear; clc; close all;
startup_MDH;

%% Load robot data (single source of truth)
data  = robotData_6R_CAD();
MDH   = data.MDH;

% Fixed CAD offset between J1 and J2 (for visualization only)
T_12_fixed = data.T_12_fixed;

%% Home configuration (CAD-like "home" pose)
n  = size(MDH,1);
q0 = zeros(n,1);
q0(2) = 2.16510;     % rad
q0(3) = -2.1650;     % rad

%% ---- 1) CAD-correct visualization ----
disp('Plotting CAD-correct robot geometry...');
plotRobotMDH_CAD(MDH, q0, T_12_fixed, true, 0.08);

%% ---- 2) Forward kinematics (pure MDH math + tool) ----
[~, T_all] = fkineMDH_all(MDH, q0);
T06 = T_all(:,:,end) * data.T_tool;

disp('End-effector pose at home (pure MDH + tool):');
disp(T06);

%% ---- 3) Jacobian + manipulability ----
J = jacobianMDH(MDH, q0);
[w, kappa] = manipulabilityMDH(MDH, q0);

fprintf('Manipulability w = %.4g\n', w);
fprintf('Condition number κ = %.4g\n', kappa);

%% ---- 4) Workspace ----
disp('Computing workspace...');
workspaceMDH(MDH, data.qmin, data.qmax, 3000);

%% ---- 5) Joint-space trajectory ----
% Target pose (you can tune these joint angles)
qf = [ pi;
       pi - 2.165;
      -pi/2;
       0;
       0;
       pi/2 ];

tf = 4;           % total motion time [s]
dt = 0.05;        % time step [s]

[t, Q, Qd, Qdd] = jointTrajQuinticMDH(q0, qf, tf, dt);

%% ---- 6) Optional time scaling (velocity / acceleration limits) ----
if isfield(data,'vmax') && isfield(data,'amax')
    [t, Q, Qd, Qdd, scale] = timeScaleVelAccMDH(t, Q, data.vmax, data.amax);
    fprintf('Time scaling factor applied: %.3f\n', scale);
    dt = mean(diff(t));
end

%% ---- 7) Animations ----
disp('Animating trajectory...');
animateRobotTrajMDH(MDH, Q, dt, false);
animateRobotTrajManipMDH(MDH, Q, dt);

%% ---- 8) Logging + dashboard plots ----
log = logTrajMDH(MDH, t, Q, Qd, Qdd);
plotLogMDH(log);

%% ---- 9) Gravity torques for payload (Jacobian-based, no RNE) ----
% We approximate a payload acting at the wrist center (frame {6} origin).
% Tool offset (85 mm along +Z6) is small and can be neglected for homework.

m_payload = 1.0;       % [kg] payload mass (adapt as needed)
g        = 9.81;       % [m/s^2]

% 9a) Static gravity torque at home pose q0
tau_g0 = staticTorquePayloadMDH(MDH, q0, m_payload, g);   % n x 1 column
disp('Static gravity torques at home q0 (payload only) [Nm]:');
disp(tau_g0);

tau_g1 = staticTorquePayloadMDH(MDH, qf, m_payload, g);   % n x 1 column
disp('Static gravity torques at work position qf (payload only) [Nm]:');
disp(tau_g1);
% 9b) Gravity torque along the trajectory
Tau_g = gravityTorqueTrajectoryMDH(MDH, Q, m_payload, g);   % N x n

% 9c) Plot torque profiles (payload gravity contributions)
figure;
for i = 1:n
    subplot(3,2,i); hold on; grid on;
    plot(t, Tau_g(:,i), 'LineWidth', 1.5);
    xlabel('t [s]');
    ylabel(sprintf('\\tau_%d [Nm]', i));
    title(sprintf('Joint %d gravity torque (payload)', i));
end
sgtitle('Joint gravity torques along trajectory (payload)');

disp('Demo completed successfully ✅');

