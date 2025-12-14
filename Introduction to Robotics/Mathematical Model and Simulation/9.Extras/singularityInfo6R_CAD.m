function info = singularityInfo6R_CAD(MDH, q, T_fixed, tol)
% SINGULARITYINFO6R_CAD  Singularities for 6R: wrist vs arm classification.

    if nargin < 4, tol = 1e-6; end
    q = q(:);

    J = jacobianMDH_CAD(MDH, q, T_fixed);
    s = svd(J);

    info = struct();
    info.sigma = s;
    info.sigma_min = min(s);
    info.kappa = cond(J);
    info.rankJ = rank(J, tol);

    info.isSingular = info.sigma_min < tol;
    info.nearSingular = (info.sigma_min < 1e-3) || (info.kappa > 500);

    % heuristic split: arm (1:3), wrist (4:6)
    J_arm = J(:,1:3);
    J_wrist = J(:,4:6);

    info.rankArm = rank(J_arm, tol);
    info.rankWrist = rank(J_wrist, tol);

    if ~info.nearSingular
        info.type = "Regular (not near singular)";
    elseif info.rankWrist < 3
        info.type = "Wrist singularity (axes 4–6 alignment / q5≈0 or π)";
    elseif info.rankArm < 3
        info.type = "Arm singularity (shoulder/elbow / reach boundary)";
    else
        info.type = "Near singular (global conditioning)";
    end
end
