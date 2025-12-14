function log = logTrajMDH(MDH, t, Q, Qd, Qdd)
% LOGTRAJMDH  Log kinematic and dexterity data along a joint trajectory.
%
%   log = logTrajMDH(MDH, t, Q)
%   log = logTrajMDH(MDH, t, Q, Qd, Qdd)
%
%   Inputs:
%     MDH : n x 5 MDH parameter table
%     t   : 1 x N time vector
%     Q   : n x N joint positions
%     Qd  : (optional) n x N joint velocities
%     Qdd : (optional) n x N joint accelerations
%
%   Output (struct fields):
%     log.t     : time vector
%     log.Q     : joint positions
%     log.Qd    : joint velocities
%     log.Qdd   : joint accelerations
%     log.T_end : 4 x 4 x N end-effector poses
%     log.p_end : 3 x N end-effector positions
%     log.J     : 6 x n x N Jacobians
%     log.w     : 1 x N manipulability index
%     log.kappa : 1 x N Jacobian condition number

    if nargin < 4, Qd = []; end
    if nargin < 5, Qdd = []; end

    [n, N] = size(Q);

    if numel(t) ~= N
        error('Length of time vector t must match number of columns in Q.');
    end
    if size(MDH,1) ~= n
        error('Number of joints in Q must match MDH table.');
    end

    % Preallocate
    T_end = zeros(4,4,N);
    p_end = zeros(3,N);
    J_all = zeros(6,n,N);
    w     = zeros(1,N);
    kappa = zeros(1,N);

    % Log trajectory
    for k = 1:N
        qk = Q(:,k);

        [~, T_all] = fkineMDH_all(MDH, qk);
        Tk = T_all(:,:,end);

        T_end(:,:,k) = Tk;
        p_end(:,k)   = Tk(1:3,4);

        Jk           = jacobianMDH(MDH, qk);
        J_all(:,:,k) = Jk;

        [wk, kk] = manipulabilityMDH(MDH, qk);
        w(k)     = wk;
        kappa(k) = kk;
    end

    % Output struct
    log = struct();
    log.t     = t;
    log.Q     = Q;
    log.Qd    = Qd;
    log.Qdd   = Qdd;
    log.T_end = T_end;
    log.p_end = p_end;
    log.J     = J_all;
    log.w     = w;
    log.kappa = kappa;
end
