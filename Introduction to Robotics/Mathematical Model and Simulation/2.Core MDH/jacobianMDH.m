function J = jacobianMDH(MDH, q)
% JACOBIANMDH  Geometric Jacobian for an MDH serial robot (base frame).
%
%   J = jacobianMDH(MDH, q)
%
%   Inputs:
%     MDH : n x 5 MDH parameter table
%     q   : n x 1 joint variable vector
%
%   Output:
%     J   : 6 x n geometric Jacobian expressed in the base frame
%           J = [Jv; Jw]
%
%   Notes:
%     - Uses Craig's Modified DH convention
%     - Valid for revolute and prismatic joints
%     - Linear and angular velocities are expressed in the base frame

    n = size(MDH,1);

    % Forward kinematics
    [~, T_all] = fkineMDH_all(MDH, q);

    % End-effector position
    Tn  = T_all(:,:,n);
    p_e = Tn(1:3,4);

    % Preallocate Jacobian
    J = zeros(6,n);

    % Loop over joints
    for i = 1:n
        if i == 1
            T_prev = eye(4);
        else
            T_prev = T_all(:,:,i-1);
        end

        z_i = T_prev(1:3,3);   % z-axis of frame {i-1}
        p_i = T_prev(1:3,4);   % origin of frame {i-1}
        type = MDH(i,5);

        if type == 0           % revolute joint
            Jv = cross(z_i, p_e - p_i);
            Jw = z_i;
        else                   % prismatic joint
            Jv = z_i;
            Jw = [0; 0; 0];
        end

        J(:,i) = [Jv; Jw];
    end
end
