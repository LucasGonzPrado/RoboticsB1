function [T_sym, T_each, J_sym, Jv_sym, Jw_sym, q, params] = symbolic_6R_CAD()
% SYMBOLIC_6R_CAD  Symbolic FK and geometric Jacobian for your 6R CAD robot.
%
%   [T_sym, T_each, J_sym, Jv_sym, Jw_sym, q, params] = symbolic_6R_CAD()
%
% Uses the MDH table (Craig convention):
%
%   MDH(i,:) = [theta0_i, d_i, a_{i-1}, alpha_{i-1}, type_i]
%   type_i   = 0 (revolute), 1 (prismatic)
%
% Layout (all revolute):
%   J1: z1 || z0
%   J2: z2 ⟂ z1          (alpha1 = +pi/2)
%   J3: z3 || z2         (alpha2 = 0)
%   J4: start wrist,     (alpha3 = +pi/2)
%   J5: wrist,           (alpha4 = -pi/2)
%   J6: wrist,           (alpha5 = +pi/2)
%
% Geometry parameters are symbolic:
%   d1_base, d3, a2, a3
%
% OUTPUTS
%   q      : 6×1 symbolic joint variables [q1..q6]
%   params : struct with symbolic dimensions (d1_base, d3, a2, a3)
%   T_sym  : 4×4 symbolic homogeneous transform ^0T_6(q)
%   T_each : 4×4×6 symbolic transforms ^0T_i(q), i = 1..6
%   Jv_sym : 3×6 symbolic linear Jacobian
%   Jw_sym : 3×6 symbolic angular Jacobian
%   J_sym  : 6×6 symbolic geometric Jacobian [Jv; Jw]
%
% REQUIREMENTS
%   - Symbolic Math Toolbox
%   - MDH_sym_A.m on your path (symbolic MDH link transform)
%
% NOTE
%   This uses pure MDH (no T_12_fixed and no tool offset). Those CAD
%   corrections are for visualization / numeric FK, not for the abstract
%   kinematic chain of joint axes.

    % ---------------------------------------------------------------------
    % 1) Symbolic joint variables and geometric parameters
    % ---------------------------------------------------------------------
    syms q1 q2 q3 q4 q5 q6 real
    q = [q1; q2; q3; q4; q5; q6];

    % Symbolic link dimensions (you can later subs numeric CAD values)
    syms d1_base d3 a2 a3 real

    params = struct('d1_base', d1_base, ...
                    'd3',      d3,      ...
                    'a2',      a2,      ...
                    'a3',      a3);

    % ---------------------------------------------------------------------
    % 2) MDH table for this specific 6R robot
    % ---------------------------------------------------------------------
    % MDH(i,:) = [theta0,   d,        a,     alpha,   type]
    MDH = [ ...
        0,   d1_base,    0,      0,        0;   % J1
        0,   0,          0,     +pi/2,     0;   % J2
        0,   d3,         a2,     0,        0;   % J3
        0,   0,          a3,    +pi/2,     0;   % J4
        0,   0,          0,     -pi/2,     0;   % J5
        0,   0,          0,     +pi/2,     0];  % J6

    n = size(MDH,1);  % = 6

    % ---------------------------------------------------------------------
    % 3) Symbolic forward kinematics ^0T_6(q)
    % ---------------------------------------------------------------------
    T = sym(eye(4));
    T_each = sym(zeros(4,4,n));

    for i = 1:n
        theta0 = MDH(i,1);
        d0     = MDH(i,2);
        a      = MDH(i,3);
        alpha  = MDH(i,4);
        type   = MDH(i,5);

        if type == 0
            % revolute
            theta = theta0 + q(i);
            d     = d0;
        else
            % prismatic (not used here, but kept for completeness)
            theta = theta0;
            d     = d0 + q(i);
        end

        A_i = MDH_sym_A(theta, d, a, alpha);  % symbolic MDH link
        T   = T * A_i;
        T_each(:,:,i) = T;
    end

    % Full FK
    T_sym = simplify(T, 'Steps', 50);

    % ---------------------------------------------------------------------
    % 4) Symbolic geometric Jacobian in base frame
    % ---------------------------------------------------------------------
    p_e = T_sym(1:3,4);       % end-effector position

    Jv_sym = sym(zeros(3,n));
    Jw_sym = sym(zeros(3,n));

    for i = 1:n
        if i == 1
            T_prev = sym(eye(4));   % base frame
        else
            T_prev = T_each(:,:,i-1);
        end

        z_i = T_prev(1:3,3);   % joint axis i in base frame
        p_i = T_prev(1:3,4);   % origin of frame i in base frame
        type = MDH(i,5);

        if type == 0
            % revolute joint
            Jv_sym(:,i) = simplify( cross(z_i, p_e - p_i) );
            Jw_sym(:,i) = z_i;
        else
            % prismatic joint
            Jv_sym(:,i) = z_i;
            Jw_sym(:,i) = sym([0;0;0]);
        end
    end

    J_sym = [Jv_sym; Jw_sym];

    % (Optional) light simplification
    J_sym   = simplify(J_sym,   'Steps', 20);
    Jv_sym  = simplify(Jv_sym,  'Steps', 20);
    Jw_sym  = simplify(Jw_sym,  'Steps', 20);
    T_each  = simplify(T_each,  'Steps', 10);
end
