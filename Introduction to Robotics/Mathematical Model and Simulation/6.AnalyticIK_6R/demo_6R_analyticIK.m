%% demo_6R_analyticIK.m
% ---------------------------------------------------------
% Demonstration of analytic inverse kinematics for a 6R
% spherical-wrist robot using Peter Corke's Robotics Toolbox.
%
% This script:
%   1) Builds a standard-DH 6R robot (UR5-like)
%   2) Computes FK for a test joint configuration
%   3) Solves IK analytically using ikine6s
%   4) Verifies FK(IK(FK(q))) ≈ FK(q)
%   5) Plots the robot
%
% Requirements:
%   - Peter Corke Robotics Toolbox (SerialLink, Link, ikine6s)
%   - create6R_robot.m
%   - ikine6R_analytic.m
% ---------------------------------------------------------

clear; clc; close all;

%% 1) Create 6R robot (standard DH, spherical wrist)
robot = create6R_robot();

disp('6R robot created (standard DH, spherical wrist).');
disp(robot);

%% 2) Define a test joint configuration (rad)
q_true = [ ...
    deg2rad(20);   % q1
    deg2rad(-40);  % q2
    deg2rad(30);   % q3
    deg2rad(10);   % q4
    deg2rad(25);   % q5
    deg2rad(-15)]; % q6

disp('True joint configuration q_true (rad):');
disp(q_true.');

%% 3) Forward kinematics
T_target = robot.fkine(q_true);

disp('Target end-effector pose T_0^6:');
disp(T_target.T);

%% 4) Analytic inverse kinematics
% Default configuration branch
q_sol = ikine6R_analytic(robot, T_target);

disp('Analytic IK solution q_sol (rad):');
disp(q_sol.');

%% 5) Verify solution with FK
T_check = robot.fkine(q_sol);

% Pose error (matrix difference norm)
errT = norm(T_target.T - T_check.T, 'fro');

fprintf('||T_target - T_fk(q_sol)||_F = %.3e\n', errT);

% Joint error (note: IK may return a different equivalent solution branch)
errq = norm(wrapToPi(q_sol - q_true));
fprintf('||wrapToPi(q_sol - q_true)||_2 = %.3e\n', errq);

%% 6) Plot robot at true config and IK solution
figure; robot.plot(q_true.');
title('Robot at q\_true');

figure; robot.plot(q_sol.');
title('Robot at q\_sol (analytic IK)');
