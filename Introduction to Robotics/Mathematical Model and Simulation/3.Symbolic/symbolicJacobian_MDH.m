function [Jv_sym, Jw_sym, J_sym, q] = symbolicJacobian_MDH(MDH)
% SYMBOLICJACOBIAN_MDH  Symbolic geometric Jacobian (MDH convention).
%
%   [Jv_sym, Jw_sym, J_sym, q] = symbolicJacobian_MDH(MDH)
%
%   Input:
%     MDH : n x 5 numeric MDH parameter table
%
%   Outputs:
%     q      : n x 1 symbolic joint variables
%     Jv_sym : 3 x n symbolic linear velocity Jacobian
%     Jw_sym : 3 x n symbolic angular velocity Jacobian
%     J_sym  : 6 x n symbolic geometric Jacobian [Jv; Jw]
%
%   Notes:
%     - Jacobian is expressed in the base frame
%     - Uses Craig's Modified DH convention
%     - Supports revolute and prismatic joints

    if ~isnumeric(MDH)
        error('MDH table must be numeric.');
    end

    n = size(MDH,1);

    % Symbolic forward kinematics
    [T_sym, T_each_sym, q] = symbolicFK_MDH(MDH);

    % End-effector position
    p_e = T_sym(1:3,4);

    % Preallocate
    Jv_sym = sym(zeros(3,n));
    Jw_sym = sym(zeros(3,n));

    % Build Jacobian
    for i = 1:n
        if i == 1
            T_prev = sym(eye(4));
        else
            T_prev = T_each_sym(:,:,i-1);
        end

        z_i = T_prev(1:3,3);
        p_i = T_prev(1:3,4);
        type = MDH(i,5);

        if type == 0          % revolute joint
            Jv_sym(:,i) = simplify(cross(z_i, p_e - p_i));
            Jw_sym(:,i) = z_i;
        else                  % prismatic joint
            Jv_sym(:,i) = z_i;
            Jw_sym(:,i) = sym([0;0;0]);
        end
    end

    % Full Jacobian
    J_sym = [Jv_sym; Jw_sym];
    simplify(J_sym)
end
