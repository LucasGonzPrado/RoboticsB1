function startup_MDH()
% STARTUP_MDH  Add MDH toolbox folders to MATLAB path.

    toolboxFolder = fileparts(mfilename('fullpath'));
    addpath(genpath(toolboxFolder));

    fprintf('✅ MDH Toolbox path added.\n');
end
