function plotKinematicDiagram_6R_CAD(q)
% PLOTKINEMATICDIAGRAM_6R_CAD
%   Kinematic diagram of the CAD-based 6R robot, with:
%     - Link stick figure
%     - Frames {0}..{6}
%     - Cylindrical joints around each z-axis (revolute)
%
%   plotKinematicDiagram_6R_CAD()
%   plotKinematicDiagram_6R_CAD(q)
%
%   Uses:
%     - robotData_6R_CAD.m
%     - fkineMDH_all.m

    % ---------------------------------------------------------------------
    % 1) Load robot data and define configuration
    % ---------------------------------------------------------------------
    data = robotData_6R_CAD();
    MDH  = data.MDH;
    T_12_fixed = data.T_12_fixed;

    n = size(MDH,1);

    if nargin < 1 || isempty(q)
        q = zeros(n,1);
        q(2) = 2.1651;
        q(3) = -2.1650;
    else
        q = q(:);
        if numel(q) ~= n
            error('q must be %d x 1.', n);
        end
    end

    % ---------------------------------------------------------------------
    % 2) Standard MDH FK to get T^0_1
    % ---------------------------------------------------------------------
    [~, T_all_MDH] = fkineMDH_all(MDH, q);
    T1 = T_all_MDH(:,:,1);      % ^0T_1 (pure MDH)

    % ---------------------------------------------------------------------
    % 3) Build CAD-corrected transforms ^0T_i (i = 0..6)
    %    Insert T_12_fixed between frame {1} and joint 2.
    % ---------------------------------------------------------------------
    T_list = cell(n+1,1);       % frames {0}..{6}
    T_list{1} = eye(4);         % base
    T_list{2} = T1;             % frame {1}

    Tprev = T1 * T_12_fixed;    % from {0} to "true" J2 position

    for i = 2:n
        A_i   = relativeMDH_A(MDH(i,:), q(i));
        Tprev = Tprev * A_i;    % ^0T_i
        T_list{i+1} = Tprev;    % store as frame {i}
    end

    % Collect frame origins
    P = zeros(3, n+1);
    for i = 1:n+1
        P(:,i) = T_list{i}(1:3,4);
    end

    % ---------------------------------------------------------------------
    % 4) Plot: links + frames + joint cylinders
    % ---------------------------------------------------------------------
    figure; hold on; grid on; view(3);

    % Stick figure
    plot3(P(1,:), P(2,:), P(3,:), '-o', ...
          'LineWidth', 2, 'MarkerSize', 6);
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    axis equal;
    title('Kinematic diagram – 6R CAD-based robot (with joint cylinders)');

    % --- parameters for frames & cylinders ---
    axisLen      = 0.08;   % axis length for triads
    jointRadius  = 0.02;   % [m] approximate radius of joint cylinder
    jointHeight  = 0.05;   % [m] height of joint cylinder along +z_i
    cylResolution = 20;    % number of points around circle

    % Plot frames and joint cylinders
    for i = 1:n+1
        Ti = T_list{i};
        o  = Ti(1:3,4);
        R  = Ti(1:3,1:3);

        % ---- Coordinate frame triads ----
        % X axis
        quiver3(o(1), o(2), o(3), ...
                axisLen*R(1,1), axisLen*R(2,1), axisLen*R(3,1), ...
                'LineWidth', 1.3, 'Color', [1 0 0]);
        % Y axis
        quiver3(o(1), o(2), o(3), ...
                axisLen*R(1,2), axisLen*R(2,2), axisLen*R(3,2), ...
                'LineWidth', 1.3, 'Color', [0 0.6 0]);
        % Z axis
        quiver3(o(1), o(2), o(3), ...
                axisLen*R(1,3), axisLen*R(2,3), axisLen*R(3,3), ...
                'LineWidth', 1.3, 'Color', [0 0 1]);

        % Frame label {0}..{6}
        text(o(1), o(2), o(3), ...
             sprintf('{%d}', i-1), ...
             'FontSize', 10, 'FontWeight', 'bold', ...
             'HorizontalAlignment', 'left', ...
             'VerticalAlignment', 'bottom');

        % ---- Cylindrical joint body (for frames {1}..{6}) ----
        if i > 1   % joints 1..6
            % Cylinder defined in local frame of joint i-1:
            % base at origin, axis along +z of that frame
            [Xc,Yc,Zc] = cylinder(jointRadius, cylResolution);
            Zc = Zc * jointHeight;   % from 0 to jointHeight

            % Local points (3 x N)
            ptsLocal = [Xc(:).'; Yc(:).'; Zc(:).'];

            % Transform to world with the SAME Ti (frame {i-1})
            % For joint i (i=1..6), the axis is z_i in frame {i}
            % but for a simple diagram, aligning cylinder with z of T_list{i}
            R_joint = R;
            p_joint = o;

            ptsWorld = R_joint * ptsLocal + p_joint;

            Xw = reshape(ptsWorld(1,:), size(Xc));
            Yw = reshape(ptsWorld(2,:), size(Yc));
            Zw = reshape(ptsWorld(3,:), size(Zc));

            surf(Xw, Yw, Zw, ...
                 'FaceColor', [0.7 0.7 0.7], ...
                 'EdgeColor', 'none', ...
                 'FaceAlpha', 0.8);
        end
    end

    % Nice viewing limits
    padding = 0.05;
    xmin = min(P(1,:)) - padding;
    xmax = max(P(1,:)) + padding;
    ymin = min(P(2,:)) - padding;
    ymax = max(P(2,:)) + padding;
    zmin = min(P(3,:)) - padding;
    zmax = max(P(3,:)) + padding;
    xlim([xmin xmax]);
    ylim([ymin ymax]);
    zlim([zmin zmax]);

    view(135, 25);
    camlight;
    lighting gouraud;
end

% -------------------------------------------------------------------------
function A = relativeMDH_A(row, qi)
% RELATIVEMDH_A  Single-joint MDH transform (Craig convention).
%   row = [theta0, d, a, alpha, type]
%   qi  = joint variable

    theta0 = row(1);
    d0     = row(2);
    a      = row(3);
    alpha  = row(4);
    type   = row(5);

    if type == 0
        theta = theta0 + qi;   % revolute
        d     = d0;
    else
        theta = theta0;        % prismatic
        d     = d0 + qi;
    end

    cth = cos(theta);  sth = sin(theta);
    cal = cos(alpha);  sal = sin(alpha);

    A = [ cth,       -sth,        0,      a;
          sth*cal,   cth*cal,    -sal,   -d*sal;
          sth*sal,   cth*sal,     cal,    d*cal;
          0,         0,           0,      1      ];
end
