clearvars -except ans; %#ok<CLVAR>

toolboxFolder = fileparts(mfilename('fullpath'));
addpath(genpath(toolboxFolder));

fprintf('✅ MDH Toolbox added to path:\n   %s\n', toolboxFolder);
