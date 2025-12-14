function plotFramesMDH(MDH, q, scale)
% PLOTFRAMESMDH  Plot base and joint coordinate frames (MDH convention).
%
%   plotFramesMDH(MDH, q)
%   plotFramesMDH(MDH, q, scale)
%
%   Inputs:
%     MDH   : n x 5 MDH parameter table
%     q     : n x 1 joint configuration
%     scale : (optional) scale factor for frame axes (default = 0.1)
%
%   Notes:
%     - Frames correspond to MDH frames {0}, {1}, ..., {n}
%     - Requires Peter Corke's Robotics Toolbox (trplot)

    if nargin < 3, scale = 0.1; end

    % Forward kinematics
    [~, T_all] = fkineMDH_all(MDH, q);
    n = size(MDH,1);

    % Check for Robotics Toolbox
    if exist('trplot','file') ~= 2
        error('trplot (Robotics Toolbox) not found.');
    end

    % Plot frames
    figure; hold on; grid on; view(3);
    xlabel('X'); ylabel('Y'); zlabel('Z');

    trplot(eye(4), 'frame', '0', 'length', scale);
    for i = 1:n
        Ti = T_all(:,:,i);
        trplot(Ti, 'frame', num2str(i), 'length', scale);
    end

    axis equal;
    title('MDH Frames');
end
