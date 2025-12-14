function info = checkSingularity(MDH, q)
    J = jacobianMDH(MDH, q);
    s = svd(J);

    info.isSingular = min(s) < 1e-6;
    info.sigma_min  = min(s);
    info.rank       = rank(J);
    info.kappa      = cond(J);
end
