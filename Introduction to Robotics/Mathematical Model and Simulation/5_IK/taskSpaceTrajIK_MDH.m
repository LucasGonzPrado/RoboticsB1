function [t, Q] = taskSpaceTrajIK_MDH(MDH, q0, p_goal, tf, dt)
% TASKSPACETRAJIK_MDH  Straight-line Cartesian position trajectory + numerical IK.
%
%   [t, Q] = taskSpaceTrajIK_MDH(MDH, q0, p_goal, tf, dt)
%
%   Inputs:
%     MDH    : n x 5 MDH table
%     q0     : n x 1 initial joint configuration
%     p_goal : 3 x 1 desired end-effector position (base frame)
%     tf     : total time [s]
%     dt     : timestep [s]
%
%   Outputs:
%     t  : 1 x N time vector
%     Q  : n x N joint trajectory
%
%   Notes:
%     - Orientation is held constant (equal to initial orientation)
%     - Uses previous IK solution as the next initial guess for robustness

    q0 = q0(:);
    p_goal = p_goal(:);

    if tf <= 0 || dt <= 0
        error('tf and dt must be positive.');
    end
    if numel(p_goal) ~= 3
        error('p_goal must be a 3x1 vector.');
    end
    if size(MDH,1) ~= numel(q0)
        error('q0 length must match number of joints in MDH.');
    end

    % Initial end-effector pose
    [~, T_all0] = fkineMDH_all(MDH, q0);
    T0 = T_all0(:,:,end);
    p0 = T0(1:3,4);
    R0 = T0(1:3,1:3);   % keep orientation fixed

    % Time vector
    N = floor(tf/dt) + 1;
    t = linspace(0, tf, N);

    n = numel(q0);
    Q = zeros(n, N);

    q_prev = q0;

    for k = 1:N
        tau = (k-1)/(N-1);                    % normalized time in [0,1]
        s = 10*tau^3 - 15*tau^4 + 6*tau^5;    % quintic time scaling

        % Straight line in Cartesian position
        p_k = p0 + s*(p_goal - p0);

        % Target pose (fixed orientation)
        T_target = eye(4);
        T_target(1:3,1:3) = R0;
        T_target(1:3,4)   = p_k;

        % Numerical IK tracking
        [q_new, ok] = ikineMDH(MDH, q_prev, T_target, 150);
        if ~ok
            warning('IK failed at step %d; holding previous q.', k);
            q_new = q_prev;
        end

        Q(:,k) = q_new;
        q_prev = q_new;
    end
end
