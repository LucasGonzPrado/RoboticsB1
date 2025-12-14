function [isCollide, minZ] = checkCollisionGroundMDH(MDH, q, zMin)
% CHECKCOLLISIONGROUNDMDH  Ground-plane collision check (MDH robot).
%
%   [isCollide, minZ] = checkCollisionGroundMDH(MDH, q, zMin)
%
%   Inputs:
%     MDH  : n x 5 MDH table
%     q    : n x 1 joint configuration
%     zMin : ground height threshold (default 0)
%
%   Outputs:
%     isCollide : true if any joint or EE is below zMin
%     minZ      : minimum z-value among base, joints, and EE

    if nargin < 3, zMin = 0; end

    q = q(:);
    n = size(MDH,1);

    if numel(q) ~= n
        error('q length must match number of joints.');
    end
    if ~isscalar(zMin)
        error('zMin must be a scalar.');
    end

    [~, T_all] = fkineMDH_all(MDH, q);

    % Collect z-coordinates (base + all joints)
    zVals = zeros(1, n+1);
    zVals(1) = 0;   % base frame assumed at z = 0

    for i = 1:n
        zVals(i+1) = T_all(3,4,i);
    end

    minZ = min(zVals);
    isCollide = (minZ < zMin);
end
