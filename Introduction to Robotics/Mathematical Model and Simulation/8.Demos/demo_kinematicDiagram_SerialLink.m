function demo_kinematicDiagram_SerialLink()
% DEMO_KINEMATICDIAGRAM_SERIALLINK
%   Example of plotting the 6R CAD-based robot using SerialLink.plot,
%   which shows revolute joints as cylinders / standard RTB style.
%
%   Requirements:
%     - Peter Corke Robotics Toolbox on the MATLAB path
%     - robotData_6R_CAD.m
%     - createSerialLink_6R_CAD.m

    startup_MDH;  % ensure toolbox folders are on the path

    % Build SerialLink model from MDH
    robotSL = createSerialLink_6R_CAD();
    disp('SerialLink model:');
    disp(robotSL);

    % CAD-like "home" configuration
    q0 = zeros(1,6);
    q0(2) = 2.1651;
    q0(3) = -2.1650;

    % Workspace bounds for nicer plot (tune if you want)
    ws = [-0.5 0.5  -0.5 0.5   0 0.8];

    figure;
    robotSL.plot(q0, ...
        'workspace', ws, ...
        'noarrow', ...      % hide joint axis arrows
        'noname', ...       % hide link names in 3D
        'noshadow', ...     % cleaner floor
        'jointdiam', 0.08); % tweak joint diameter

    title('SerialLink kinematic diagram – 6R CAD-based MDH model');
    grid on;
end
