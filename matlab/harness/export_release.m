function zipFile = export_release(version, outDir)
%EXPORT_RELEASE Package the harness for an older MATLAB release (to share it outside the repo).
%   export_release()                    R2023a package in <repo>/dist (opens in R2023a and newer)
%   export_release('R2023b')            another target release
%   export_release('R2023a', outDir)    another output folder
%
%   Copies matlab/harness and matlab/model (same layout, so harness_setup still finds the model),
%   re-saves every .slx in the target release's format with Simulink.exportToVersion, leaves out
%   build caches (slprj, .slxc) and the repo-only run exporter, and zips the result. The source
%   files are only read, never changed. Needs on the receiving side: MATLAB, Simulink, Simscape,
%   Simscape Multibody, Control System Toolbox (LQR design).
arguments
    version (1,:) char = 'R2023a'
    outDir (1,:) char = ''
end
root = harness_setup();
repo = fileparts(fileparts(root));
if isempty(outDir), outDir = fullfile(repo, 'dist'); end
name = ['furuta_harness_' version];
pkg = fullfile(outDir, name);
if isfolder(pkg), rmdir(pkg, 's'); end
mkdir(pkg);

copy_tree(fullfile(repo, 'matlab', 'model'), fullfile(pkg, 'model'));
copy_tree(root, fullfile(pkg, 'harness'));

% re-save every model in the target format (the copies above are R2026a files)
bdclose('all');
slx = dir(fullfile(root, '**', '*.slx'));
slx = slx(~contains({slx.folder}, [filesep 'slprj']));
for k = 1:numel(slx)
    src = fullfile(slx(k).folder, slx(k).name);
    rel = extractAfter(src, numel(root) + 1);
    dst = fullfile(pkg, 'harness', rel);
    delete(dst);
    [~, mdl] = fileparts(src);
    load_system(src);
    Simulink.exportToVersion(mdl, dst, upper(version), 'AllowPrompt', false);
    close_system(mdl, 0);
    fprintf('  %-55s -> %s\n', rel, version);
end

zipFile = fullfile(outDir, [name '.zip']);
if isfile(zipFile), delete(zipFile); end
zip(zipFile, {'harness', 'model'}, pkg);
fprintf('Packaged for %s: %s\nUnzip, then in MATLAB: cd harness; harness_setup; run_sim(...) (see harness/README.md)\n', ...
    version, zipFile);
end

function copy_tree(src, dst)
% Copy a folder without build caches, editor backups or repo-only tools.
skipDir = {'slprj', '.git'};
skipExt = {'.slxc', '.asv', '.mexw64'};
skipFile = {'export_runs.py'};          % needs the repo's data/ and Python app; the .mat runs are included
mkdir(dst);
items = dir(src);
for k = 1:numel(items)
    it = items(k);
    if any(strcmp(it.name, {'.', '..'})), continue, end
    s = fullfile(src, it.name);
    d = fullfile(dst, it.name);
    if it.isdir
        if ~any(strcmp(it.name, skipDir)), copy_tree(s, d); end
    else
        [~, ~, ext] = fileparts(it.name);
        if ~any(strcmpi(ext, skipExt)) && ~any(strcmp(it.name, skipFile)), copyfile(s, d); end
    end
end
end
