function root = harness_setup()
%HARNESS_SETUP Put the harness, its library, every controller folder and the identified model on the path.
%   root = harness_setup() returns the harness folder. Safe to call repeatedly.
root = fileparts(mfilename('fullpath'));
addpath(root, fullfile(root, 'lib'), fullfile(root, 'plant3d'), fullfile(root, '..', 'model'));
addpath(genpath(fullfile(root, 'controllers')));
end
