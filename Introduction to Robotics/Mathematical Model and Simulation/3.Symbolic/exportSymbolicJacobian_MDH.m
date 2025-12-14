function exportSymbolicJacobian_MDH(MDH)
% EXPORTSYMBOLICJACOBIAN_MDH  Export symbolic MDH Jacobian to numeric function.
%
%   exportSymbolicJacobian_MDH(MDH)
%
%   This function:
%     1) Builds symbolic Jacobian using symbolicJacobian_MDH
%     2) Converts it to a fast numeric MATLAB function
%     3) Saves it as jacobianMDH_symbolic.m
%
%   Output function usage:
%     J = jacobianMDH_symbolic(q)
%
%   where q is an n x 1 numeric joint vector.

    if ~isnumeric(MDH)
        error('MDH table must be numeric.');
    end

    % Generate symbolic Jacobian
    [~, ~, J_sym, q] = symbolicJacobian_MDH(MDH);

    % Export to numeric MATLAB function
    matlabFunction(J_sym, ...
        'Vars', {q}, ...
        'File', 'jacobianMDH_symbolic', ...
        'Optimize', true);

    fprintf('✔ Symbolic Jacobian exported to jacobianMDH_symbolic.m\n');
end
