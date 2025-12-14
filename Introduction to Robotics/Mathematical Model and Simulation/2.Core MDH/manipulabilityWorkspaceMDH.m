function manipulabilityWorkspaceMDH(MDH, qmin, qmax, N)
% MANIPULABILITYWORKSPACEMDH  Workspace colored by manipulability index.
%
%   manipulabilityWorkspaceMDH(MDH, qmin, qmax)
%   manipulabilityWorkspaceMDH(MDH, qmin, qmax, N)
%
%   Inputs:
%     MDH  : n x 5 MDH parameter table
%     qmin : n x 1 vector of joint lower limits
%     qmax : n x 1 vector of joint upper limits
%     N    : (optional) number of random samples (default = 1000)
%
%   Notes:
%     - Each workspace point corresponds to an end-effector position
%     - Color indicates Yoshikawa manipulability index w(q)
%     - Sampling is uniform in joint space

    if nargin < 4, N = 1000; end
    n = size(MDH,1);

    qmin = qmin(:);
    qmax = qmax(:);

    if numel(qmin) ~= n || numel(qmax) ~= n
        error('qmin and qmax must be length n.');
    end

    % Optional reproducibility
    % rng(0);

    pts  = zeros(N,3);
    wval = zeros(N,1);

    for k = 1:N
        q = qmin + (qmax - qmin) .* rand(n,1);
        [~, T_all] = fkineMDH_all(MDH, q);
        pts(k,:) = T_all(1:3,4,end).';
        [wk, ~] = manipulabilityMDH(MDH, q);
        wval(k) = wk;
    end

    % Plot workspace colored by manipulability
    figure; hold on; grid on; view(3);
    scatter3(pts(:,1), pts(:,2), pts(:,3), 8, wval, 'filled');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis equal;
    colormap jet;
    c = colorbar; ylabel(c, 'w (Yoshikawa)');
    title('Workspace Colored by Manipulability');
end
