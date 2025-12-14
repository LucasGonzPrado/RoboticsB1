function plotLogMDH(log)
% PLOTLOGMDH  Visualization dashboard from trajectory log.
%
%   plotLogMDH(log)
%
%   Input:
%     log : structure returned by logTrajMDH
%
%   Plots:
%     1) End-effector Cartesian trajectory
%     2) Joint positions, velocities, accelerations
%     3) Manipulability index and Jacobian condition number

    t     = log.t;
    Q     = log.Q;
    Qd    = log.Qd;
    Qdd   = log.Qdd;
    p_end = log.p_end;
    w     = log.w;
    kappa = log.kappa;

    [n, ~] = size(Q);

    %% End-effector path
    figure; hold on; grid on; view(3);
    plot3(p_end(1,:), p_end(2,:), p_end(3,:), 'LineWidth', 1.5);
    plot3(p_end(1,1),   p_end(2,1),   p_end(3,1), 'go', 'MarkerSize', 8);
    plot3(p_end(1,end), p_end(2,end), p_end(3,end), 'rx', 'MarkerSize', 8);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis equal;
    title('End-Effector Trajectory');

    %% Joint profiles
    figure;

    ax1 = subplot(3,1,1); hold on; grid on;
    title('Joint Positions'); ylabel('q');

    ax2 = subplot(3,1,2); hold on; grid on;
    title('Joint Velocities'); ylabel('q̇');

    ax3 = subplot(3,1,3); hold on; grid on;
    title('Joint Accelerations'); ylabel('q̈'); xlabel('t [s]');

    for i = 1:n
        plot(ax1, t, Q(i,:), 'DisplayName', sprintf('q_%d', i));

        if ~isempty(Qd)
            plot(ax2, t, Qd(i,:), 'DisplayName', sprintf('q̇_%d', i));
        end

        if ~isempty(Qdd)
            plot(ax3, t, Qdd(i,:), 'DisplayName', sprintf('q̈_%d', i));
        end
    end

    legend(ax1, 'show');
    if ~isempty(Qd),  legend(ax2, 'show'); end
    if ~isempty(Qdd), legend(ax3, 'show'); end

    %% Manipulability metrics
    figure;

    subplot(2,1,1); grid on;
    plot(t, w, 'LineWidth', 1.5);
    ylabel('w');
    title('Yoshikawa Manipulability Index');

    subplot(2,1,2); grid on;
    plot(t, kappa, 'LineWidth', 1.5);
    ylabel('\kappa(J)');
    xlabel('t [s]');
    title('Jacobian Condition Number');
end
