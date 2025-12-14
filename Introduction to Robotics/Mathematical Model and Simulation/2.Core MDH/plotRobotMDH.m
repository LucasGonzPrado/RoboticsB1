function plotRobotMDH(MDH, q, showFrames, frameScale)
% PLOTROBOTMDH  Plot an MDH robot as a 3D stick figure.
%
%   plotRobotMDH(MDH, q)
%   plotRobotMDH(MDH, q, showFrames, frameScale)
%
%   Inputs:
%     MDH        : n x 5 MDH parameter table
%     q          : n x 1 joint configuration
%     showFrames : (optional) true/false to plot MDH frames
%     frameScale : (optional) scale factor for frame axes
%
%   Notes:
%     - Joint positions are extracted from forward kinematics
%     - Frames correspond to MDH frames {0}, {1}, ..., {n}

    if nargin < 3, showFrames = false; end
    if nargin < 4, frameScale = 0.1; end

    % Forward kinematics
    [~, T_all] = fkineMDH_all(MDH, q);
    n = size(MDH,1);

    % Collect joint positions
    P = zeros(3,n+1);
    P(:,1) = [0;0;0];  % base origin
    for i = 1:n
        Ti = T_all(:,:,i);
        P(:,i+1) = Ti(1:3,4);
    end

    % Plot robot
    figure; hold on; grid on; view(3);
    plot3(P(1,:), P(2,:), P(3,:), '-o', ...
          'LineWidth', 2, 'MarkerSize', 6);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis equal;
    title('Robot (MDH stick figure)');

    % Plot frames if requested and available
    if showFrames && exist('trplot','file') == 2
        trplot(eye(4), 'frame', '0', 'length', frameScale);
        for i = 1:n
            Ti = T_all(:,:,i);
            trplot(Ti, 'frame', num2str(i), 'length', frameScale);
        end
    end
end
