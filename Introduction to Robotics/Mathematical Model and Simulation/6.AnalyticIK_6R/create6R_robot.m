function robot = create6R_robot()
% CREATE6R_ROBOT  UR5-like 6R robot with spherical wrist (standard DH).
%
%   robot = create6R_robot()
%
%   Notes:
%     - Uses STANDARD Denavit–Hartenberg convention
%     - Geometry ensures a spherical wrist (axes 4–6 intersect)
%     - Compatible with SerialLink.ikine6s (analytic IK)

    DOF = 6;

    % Standard DH parameters (UR5-like)
    alpha = [ pi/2,  0,       0,      pi/2,   -pi/2,   0 ];
    a     = [ 0,    -0.425, -0.39225,  0,       0,     0 ];
    d     = [ 0.089459, 0, 0, 0.10915, 0.09465, 0.0823 ];

    % Create links
    L(DOF) = Link();
    for i = 1:DOF
        L(i) = Link([0, d(i), a(i), alpha(i)], 'standard');
    end

    % Build robot
    robot = SerialLink(L, 'name', 'UR5_like_6R');
end
