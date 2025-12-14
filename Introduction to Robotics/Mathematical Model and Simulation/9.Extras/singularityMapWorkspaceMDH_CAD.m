function singularityMapWorkspaceMDH_CAD(MDH, qmin, qmax, T_fixed, N, mode)
% SINGULARITYMAPWORKSPACEMDH_CAD  Sample workspace and color by sigma_min or kappa.

    if nargin < 5, N = 4000; end
    if nargin < 6, mode = 'sigma'; end

    n = size(MDH,1);
    qmin = qmin(:); qmax = qmax(:);

    pts = zeros(N,3);
    val = zeros(N,1);

    for k = 1:N
        q = qmin + (qmax-qmin).*rand(n,1);

        [~, T_all] = fkineMDH_all_CAD(MDH, q, T_fixed);
        pts(k,:) = T_all(1:3,4,end).';

        J = jacobianMDH_CAD(MDH, q, T_fixed);

        if strcmpi(mode,'sigma')
            s = svd(J);
            val(k) = min(s);
        else
            val(k) = cond(J);
            if ~isfinite(val(k)), val(k) = 1e6; end
        end
    end

    figure; hold on; grid on; view(3);
    scatter3(pts(:,1),pts(:,2),pts(:,3),8,val,'filled');
    xlabel('X'); ylabel('Y'); zlabel('Z'); axis equal;
    colorbar;
    title(sprintf('Workspace singularity map (%s)', mode));
end
