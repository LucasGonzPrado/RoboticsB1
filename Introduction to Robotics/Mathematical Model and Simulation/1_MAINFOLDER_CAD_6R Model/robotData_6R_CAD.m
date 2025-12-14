function data = robotData_6R_CAD()
    data = struct();
    data.name = 'My6R_fromCAD';

    % ----- geometry -----
    d1_base = 31.00e-3;
    d2      = 10.00e-3;
    d3      = 20.0e-3;
    dz12    = 109.88e-3;
    dx12    = 94.60e-3;
    a2      = 280.00e-3;
    a3      = 235.00e-3;
    tool    = 85.00e-3;

    % MDH: joint axes ONLY (no CAD fudge here)
    data.MDH = [ ...
    % theta0   d         a      alpha     type
        0,   d1_base,    0,      0,        0;   % J1
        0,   0,         0,      +pi/2,     0;   % J2
        0,   d3,        a2,      0,        0;   % J3
        0,   0,         a3,     +pi/2,     0;   % J4
        0,   0,          0,     -pi/2,     0;   % J5
        0,   0,          0,     +pi/2,     0];  % J6

    % CAD adapter J1→J2
    data.T_12_fixed = [eye(3) [dx12; d2; dz12]; 0 0 0 1];

    % Joint limits
    data.qmin = deg2rad([-170; -120; -170; -190; -120; -360]);
    data.qmax = deg2rad([ 170;  120;  170;  190;  120;  360]);

    % Velocity / acceleration limits
    data.vmax = deg2rad([120;120;120;180;180;180]);
    data.amax = deg2rad([300;300;300;600;600;600]);

    % Base / tool
    data.T_base = eye(4);
    data.T_tool = [eye(3) [0;0;tool]; 0 0 0 1];

    % ====== DYNAMIC PARAMETERS (FILL FROM CAD LATER) ======
    % Masses [kg]
    data.m = [ ...
        13.0;
        11.0;
        9.0;
        3.25;
        2.25;
        1.4];  % <-- put real values here

    % COM positions in each link frame [m], columns = links
    % r = [rx1 rx2 ...; ry1 ...; rz1 ...]
    data.r = [ ...
        0,    0,    0,    0,    0,    0;   % x-coordinates
        0,    0,    0,    0,    0,    0;   % y-coordinates
       d1_base/2, dz12/2, a2/2, a3/2, 0,   0];  % rough z/x guesses

    % Inertia matrices around link frame origin (3x3x6) [kg m^2]
    data.I = zeros(3,3,6);
    % Example placeholder for link 1:
    % data.I(:,:,1) = diag([0.01, 0.01, 0.005]);
end
