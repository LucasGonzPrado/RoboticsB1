clear; clc;

% Add toolbox folders to MATLAB path
startup_MDH;   % or startup_MDH();

% Load robot model
[MDH, ~, ~] = getMDH_6R();

% --- Symbolic Forward Kinematics ---
[T_sym, T_each, q_sym] = symbolicFK_MDH(MDH);

% Symbolic end-effector position
p_sym = simplify(T_sym(1:3,4));

disp("Symbolic end-effector position p(q):");
disp(p_sym);

% --- Symbolic Jacobian ---
[Jv_sym, Jw_sym, J_sym, ~] = symbolicJacobian_MDH(MDH);

disp("Size of symbolic Jacobian [rows, cols]:");
disp(size(J_sym));
