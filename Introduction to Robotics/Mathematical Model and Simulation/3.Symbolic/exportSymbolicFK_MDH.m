function exportSymbolicFK_MDH(MDH)
% EXPORTSYMBOLICFK_MDH  Export symbolic MDH forward kinematics to numeric function.
%
%   exportSymbolicFK_MDH(MDH)
%
%   This function:
%     1) Builds symbolic forward kinematics using symbolicFK_MDH
%     2) Converts it to a fast numeric MATLAB function
%     3) Saves it as fkineMDH_symbolic.m
%
%   Output function usage:
%     T = fkineMDH_symbolic(q)
%
%   where q is an n x 1 numeric joint vector.

    if ~isnumeric(MDH)
        error('MDH table must be numeric.');
    end

    % Generate symbolic FK
    [T_sym, ~, q] = symbolicFK_MDH(MDH);

    % Export to numeric MATLAB function
    matlabFunction(T_sym, ...
        'Vars', {q}, ...
        'File', 'fkineMDH_symbolic', ...
        'Optimize', true);

    fprintf('✔ Symbolic FK exported to fkineMDH_symbolic.m\n');
end
