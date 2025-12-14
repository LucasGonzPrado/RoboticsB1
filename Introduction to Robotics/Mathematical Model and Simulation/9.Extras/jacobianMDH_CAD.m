function J = jacobianMDH_CAD(MDH, q, T_fixed)
% JACOBIANMDH_CAD  Geometric Jacobian (base frame) using CAD-aware FK.

    n = size(MDH,1);
    [~, T_all] = fkineMDH_all_CAD(MDH, q, T_fixed);
    J = zeros(6,n);

    p_e = T_all(1:3,4,end);

    for i = 1:n
        if i == 1
            T_prev = eye(4);
        else
            T_prev = T_all(:,:,i-1);
        end

        z = T_prev(1:3,3);
        p = T_prev(1:3,4);

        if MDH(i,5) == 0  % revolute
            Jv = cross(z, p_e - p);
            Jw = z;
        else              % prismatic
            Jv = z;
            Jw = [0;0;0];
        end

        J(:,i) = [Jv; Jw];
    end
end
