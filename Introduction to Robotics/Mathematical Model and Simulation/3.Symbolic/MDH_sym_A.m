function A = MDH_sym_A(theta, d, a, alpha)
% MDH_SYM_A  Symbolic Modified DH homogeneous transform (Craig convention).
%
%   A = MDH_sym_A(theta, d, a, alpha)
%
%   Inputs:
%     theta : joint angle [rad]
%     d     : link offset [m]
%     a     : link length [m]
%     alpha : link twist [rad]
%
%   Output:
%     A     : 4x4 homogeneous transform ^{i-1}T_i
%
%   Notes:
%     - Uses Craig's Modified DH convention
%     - Inputs may be symbolic or numeric

    A = [ cos(theta),            -sin(theta),             0,          a;
          sin(theta)*cos(alpha),  cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha);
          sin(theta)*sin(alpha),  cos(theta)*sin(alpha),  cos(alpha),  d*cos(alpha);
          0,                       0,                      0,          1];
end
