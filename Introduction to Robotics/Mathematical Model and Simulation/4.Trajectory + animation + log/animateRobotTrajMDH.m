function animateRobotTrajMDH(MDH, Q, dt, showFrames, frameScale)
% ANIMATEROBOTTRAJMDH  Animate an MDH robot along a joint-space trajectory.
%
%   animateRobotTrajMDH(MDH, Q, dt)
%   animateRobotTrajMDH(MDH, Q, dt, showFrames, frameScale)
%
%   Inputs:
%     MDH        : n x 5 MDH parameter table
%     Q          : n x N joint trajectory
%     dt         : animation timestep [s]
%     showFrames : (optional) true/false to plot MDH frames
%     frameScale : (optional) scale factor for frame axes

    if nargin < 4, showFrames = false; end
    if nargin < 5, frameScale = 0.08; end

    [n, N] = size(Q);
    if size(MDH,1) ~= n
        error('Number of joints in Q must match MDH table.');
    end

    % Initial configuration
    q = Q(:,1);
    [~, T_all] = fkineMDH_all(MDH, q);

    % Initial joint positions
    P = zeros(3,n+1);
    P(:,1) = [0;0;0];
    for i = 1:n
        P(:,i+1) = T_all(1:3,4,i);
    end

    % Create figure
    figure; hold on; grid on; view(3);
    hLinks = plot3(P(1,:), P(2,:), P(3,:), '-o', ...
                   'LineWidth', 2, 'MarkerSize', 6);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis equal;
    title('Robot Animation (MDH)');

    % Plot frames once (optional)
    if showFrames && exist('trplot','file') == 2
        hFrames = gobjects(n+1,1);
        hFrames(1) = trplot(eye(4), 'frame', '0', 'length', frameScale);
        for i = 1:n
            hFrames(i+1) = trplot(T_all(:,:,i), ...
                                  'frame', num2str(i), ...
                                  'length', frameScale);
        end
    else
        hFrames = [];
    end

    % Animation loop
    for k = 1:N
        q = Q(:,k);
        [~, T_all] = fkineMDH_all(MDH, q);

        for i = 1:n
            P(:,i+1) = T_all(1:3,4,i);
        end
        set(hLinks, 'XData', P(1,:), ...
                    'YData', P(2,:), ...
                    'ZData', P(3,:));

        % Update frames only if enabled
        if ~isempty(hFrames)
            delete(hFrames);
            hFrames(1) = trplot(eye(4), 'frame', '0', 'length', frameScale);
            for i = 1:n
                hFrames(i+1) = trplot(T_all(:,:,i), ...
                                      'frame', num2str(i), ...
                                      'length', frameScale);
            end
        end

        drawnow limitrate;
        pause(dt);
    end
end
