function [MDH, qmin, qmax] = getMDH_6R()
% GETMDH_6R  MDH parameters and joint limits for a 6R serial robot.
%
%   [MDH, qmin, qmax] = getMDH_6R()
%
%   This function defines a 6R serial manipulator using the
%   Modified Denavit–Hartenberg (MDH) convention (Craig).
%
%   MDH(i,:) = [theta0_i  d_i  a_{i-1}  alpha_{i-1}  type_i]
%
%   where:
%     - theta0_i : constant joint angle offset [rad]
%     - d_i      : link offset along z_i [m]
%     - a_{i-1}  : link length along x_{i-1} [m]
%     - alpha_{i-1} : link twist about x_{i-1} [rad]
%     - type_i   : joint type (0 = revolute, 1 = prismatic)
%
%   The robot consists of 6 revolute joints (6R), with a
%   spherical wrist configuration at joints 4–6.
%
%   Outputs:
%     MDH  : 6x5 MDH parameter table
%     qmin : 6x1 vector of joint lower limits [rad]
%     qmax : 6x1 vector of joint upper limits [rad]

    % -----------------------
    % Geometric parameters (meters)
    % -----------------------
    L1 = 0.30;   % base height
    L2 = 0.40;   % upper arm length
    L3 = 0.30;   % forearm length
    L4 = 0.15;   % wrist offset 1
    L5 = 0.10;   % wrist offset 2
    L6 = 0.10;   % tool length

    % -----------------------
    % MDH table
    % -----------------------
    % [theta0  d    a     alpha     type]
    MDH = [ ...
        0   L1   0     -pi/2   0;   % Joint 1 (R)
        0   0    L2     0      0;   % Joint 2 (R)
        0   0    L3     0      0;   % Joint 3 (R)
        0   L4   0     -pi/2   0;   % Joint 4 (R)
        0   L5   0      pi/2   0;   % Joint 5 (R)
        0   L6   0      0      0];  % Joint 6 (R)

    % -----------------------
    % Joint limits (radians)
    % -----------------------
    qmin = deg2rad([-170; -120; -170; -190; -120; -360]);
    qmax = deg2rad([ 170;  120;  170;  190;  120;  360]);

end
