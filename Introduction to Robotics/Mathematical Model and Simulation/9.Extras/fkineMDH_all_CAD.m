function [A_all, T_all] = fkineMDH_all_CAD(MDH, q, T_fixed)
% FKINEMDH_ALL_CAD  MDH FK with optional fixed CAD transform inserted.
%
%   [A_all, T_all] = fkineMDH_all_CAD(MDH, q, T_fixed)
%
%   T_fixed.after : integer i, insert after joint i
%   T_fixed.T     : 4x4 fixed transform
%
%   Notes:
%   - n joints = n DOF (fixed transform does not add DOF)
%   - T_all(:,:,i) remains ^0T_i, but includes fixed transforms if inserted.

    n = size(MDH,1);
    if numel(q) ~= n
        error('Length of q must equal number of MDH rows.');
    end
    q = q(:);

    A_all = zeros(4,4,n);
    T_all = zeros(4,4,n);

    T = eye(4);

    useFixed = nargin >= 3 && ~isempty(T_fixed) && isfield(T_fixed,'after') && isfield(T_fixed,'T');

    for i = 1:n
        A = mdhA_one(MDH(i,:), q(i));
        A_all(:,:,i) = A;

        T = T * A;

        if useFixed && i == T_fixed.after
            T = T * T_fixed.T;  % insert rigid CAD adapter
        end

        T_all(:,:,i) = T;
    end
end

function A = mdhA_one(row, qi)
% MDH transform for one joint row: row = [theta0 d a alpha type]
    theta0 = row(1); d0 = row(2); a = row(3); alpha = row(4); type = row(5);

    if type == 0
        theta = theta0 + qi;
        d = d0;
    else
        theta = theta0;
        d = d0 + qi;
    end

    cth = cos(theta);  sth = sin(theta);
    cal = cos(alpha);  sal = sin(alpha);

    A = [ cth,       -sth,        0,      a;
          sth*cal,   cth*cal,    -sal,   -d*sal;
          sth*sal,   cth*sal,     cal,    d*cal;
          0,         0,           0,      1 ];
end
