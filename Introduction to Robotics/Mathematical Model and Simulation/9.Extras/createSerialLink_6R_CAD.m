function robotSL = createSerialLink_6R_CAD()
% CREATESERIALLINK_6R_CAD
%   Build a SerialLink 6R robot from the CAD-based MDH table.
%
%   robotSL = createSerialLink_6R_CAD()
%
%   Uses:
%     - robotData_6R_CAD.m
%   Notes:
%     - Kinematic-only (no inertias / masses required).
%     - Uses Craig's Modified DH convention in SerialLink.

    % Load geometry + MDH
    data = robotData_6R_CAD();
    MDH  = data.MDH;
    n    = size(MDH,1);

    % Preallocate Link array
    L(1:n) = Link();

    for i = 1:n
        theta0 = MDH(i,1);   % nominal theta
        d0     = MDH(i,2);
        a      = MDH(i,3);
        alpha  = MDH(i,4);
        type   = MDH(i,5);   % 0 = revolute, 1 = prismatic

        sigma = type;        % SerialLink uses same convention

        % Kinematic Link (Modified DH)
        %
        % theta = q + offset for revolute,
        % d     = q + offset for prismatic
        %
        % Here MDH is defined as theta = theta0 + q(i) for revolute,
        % and d = d0 + q(i) for prismatic. Since we keep theta0 in the
        % constant part, we can store it as 'offset' if nonzero.
        if sigma == 0
            % Revolute joint
            L(i) = Link([0, d0, a, alpha, sigma], 'modified');
            L(i).offset = theta0;   % incorporate theta0 as offset
        else
            % Prismatic joint
            L(i) = Link([theta0, 0, a, alpha, sigma], 'modified');
            L(i).offset = d0;       % incorporate d0 as offset
        end
    end

    % Build the SerialLink robot
    robotSL = SerialLink(L, 'name', data.name);

    % Base/frame transforms (same as your MDH model)
    if isfield(data,'T_base')
        robotSL.base = data.T_base;
    end
    if isfield(data,'T_tool')
        robotSL.tool = data.T_tool;
    end
end
