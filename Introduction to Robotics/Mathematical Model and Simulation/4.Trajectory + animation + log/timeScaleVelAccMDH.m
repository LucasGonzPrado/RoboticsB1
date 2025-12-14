function [t_new, Q_new, Qd_new, Qdd_new, scale] = timeScaleVelAccMDH(t, Q, vmax, amax)
% TIMESCALEVELACCMDH  Uniform time scaling to satisfy joint vel/acc limits.
%
%   [t_new, Q_new, Qd_new, Qdd_new, scale] = timeScaleVelAccMDH(t, Q, vmax, amax)
%
%   Inputs:
%     t    : 1xN or Nx1 time vector [s], strictly increasing
%     Q    : n x N joint positions (rad or m)
%     vmax : n x 1 joint velocity limits (rad/s or m/s). Use <=0 to ignore a joint.
%     amax : n x 1 joint acceleration limits (rad/s^2 or m/s^2). Use <=0 to ignore a joint.
%
%   Outputs:
%     t_new   : scaled time vector (t_new = scale * t)
%     Q_new   : same as Q (uniform time scaling does not change positions)
%     Qd_new  : recomputed velocities vs t_new
%     Qdd_new : recomputed accelerations vs t_new
%     scale   : scalar >= 1 time scaling factor
%
%   Notes:
%     Uniform scaling factor s affects derivatives as:
%       qdot_new  = qdot / s
%       qddot_new = qddot / s^2

    % Shape inputs
    t = t(:).';                 % row
    [n, Nq] = size(Q);
    N = numel(t);

    if Nq ~= N
        error('Q must be n x N and match length(t).');
    end
    if any(diff(t) <= 0)
        error('t must be strictly increasing.');
    end

    vmax = vmax(:);
    amax = amax(:);

    if numel(vmax) ~= n || numel(amax) ~= n
        error('vmax and amax must be length n.');
    end

    % --- Compute current velocity and acceleration vs t ---
    Qd  = zeros(n, N);
    Qdd = zeros(n, N);

    for i = 1:n
        Qd(i,:)  = gradient(Q(i,:), t);
        Qdd(i,:) = gradient(Qd(i,:), t);
    end

    % --- Determine required scaling ---
    scale_v = 1;
    scale_a = 1;

    for i = 1:n
        if vmax(i) > 0
            s_vi = max(abs(Qd(i,:))) / vmax(i);
            scale_v = max(scale_v, s_vi);
        end
        if amax(i) > 0
            s_ai = sqrt(max(abs(Qdd(i,:))) / amax(i));
            scale_a = max(scale_a, s_ai);
        end
    end

    scale = max(1, max(scale_v, scale_a));

    % --- Apply scaling ---
    t_new = t * scale;
    Q_new = Q;

    % Recompute derivatives vs scaled time
    Qd_new  = zeros(n, N);
    Qdd_new = zeros(n, N);

    for i = 1:n
        Qd_new(i,:)  = gradient(Q_new(i,:), t_new);
        Qdd_new(i,:) = gradient(Qd_new(i,:), t_new);
    end
end
