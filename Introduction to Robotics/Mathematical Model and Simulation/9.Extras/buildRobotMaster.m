function robot = buildRobotMaster(data)
% BUILDROBOTMASTER  Build a unified robot struct from CAD/parameter data.
%
%   robot = buildRobotMaster(data)
%
%   Input:
%     data : struct containing (at minimum)
%            - data.name
%            - data.MDH   (n x 5, Craig MDH)
%            - data.qmin  (n x 1)
%            - data.qmax  (n x 1)
%
%   Optional fields in data:
%            - vmax  (n x 1) joint velocity limits
%            - amax  (n x 1) joint acceleration limits
%            - T_base (4 x 4 homogeneous transform)
%            - T_tool (4 x 4 homogeneous transform)
%
%   Output:
%     robot : master robot struct used consistently across:
%             - FK / Jacobian
%             - workspace analysis
%             - trajectory generation
%             - animation and logging
%
%   Notes:
%     - All units must be SI (meters, radians)
%     - MDH convention: [theta0 d a alpha type]
%     - This function does NOT modify geometry; it only packages it safely

    % -----------------------------
    % Basic validation
    % -----------------------------
    requiredFields = {'name','MDH','qmin','qmax'};
    for k = 1:numel(requiredFields)
        if ~isfield(data, requiredFields{k})
            error('buildRobotMaster:MissingField', ...
                  'Required field "%s" is missing from data.', requiredFields{k});
        end
    end

    MDH = data.MDH;
    if size(MDH,2) ~= 5
        error('MDH table must be n x 5: [theta0 d a alpha type].');
    end

    n = size(MDH,1);

    if numel(data.qmin) ~= n || numel(data.qmax) ~= n
        error('qmin and qmax must be length n (number of joints).');
    end

    % -----------------------------
    % Core robot definition
    % -----------------------------
    robot = struct();
    robot.name = data.name;

    robot.MDH  = MDH;
    robot.n    = n;

    robot.qmin = data.qmin(:);
    robot.qmax = data.qmax(:);

    % -----------------------------
    % Optional limits
    % -----------------------------
    if isfield(data,'vmax')
        robot.vmax = data.vmax(:);
    else
        robot.vmax = [];
    end

    if isfield(data,'amax')
        robot.amax = data.amax(:);
    else
        robot.amax = [];
    end

    % -----------------------------
    % Base and tool transforms
    % -----------------------------
    if isfield(data,'T_base')
        robot.T_base = data.T_base;
    else
        robot.T_base = eye(4);
    end

    if isfield(data,'T_tool')
        robot.T_tool = data.T_tool;
    else
        robot.T_tool = eye(4);
    end

    % -----------------------------
    % Metadata (useful for reports/debug)
    % -----------------------------
    robot.convention = 'MDH (Craig)';
    robot.timestamp  = datetime('now');

end
