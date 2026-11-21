function C = swingup_catch_region(varargin)
%SWINGUP_CATCH_REGION Where the LQR can take the pendulum over: the target of the swing-up.
%   C = swingup_catch_region()                         full grid, saved to swingup_catch.mat
%   C = swingup_catch_region('Part', [k n])            every n-th grid point from k (parallel MATLABs)
%   C = swingup_catch_region('Merge', n)               merge the n part files, plot, save the result
%   Options: 'Th1' [deg] (>= 0, mirrored), 'W1' [rad/s], 'T' run length [s], 'File' result file,
%            'Params' extra controller knobs (default: the design file's LQR).
%
%   Each point starts the closed loop at theta1, w1 with the arm centred and at rest, the Stateflow
%   guards opened wide so the LQR is in charge from t = 0 (the state estimator starts from zero
%   velocity, as after a hand-over without history: slightly pessimistic). Realistic sensors and
%   firmware. Held = no firmware trip and |theta1| < 10 deg during the last second (the swing-up
%   scenario's own criterion; the soft LQR still wobbles ~3 deg at 3 s).
%   The plant is symmetric, so (theta1, w1) and (-theta1, -w1) behave the same: only theta1 >= 0 is
%   run and the map is mirrored.
p = inputParser;
p.addParameter('Th1', 0:2.5:35);
p.addParameter('W1', -14:1:14);
p.addParameter('T', 3);
p.addParameter('Part', [1 1]);
p.addParameter('Merge', 0);
p.addParameter('File', fullfile(fileparts(mfilename('fullpath')), 'swingup_catch.mat'));
p.addParameter('Params', struct());
p.parse(varargin{:});
o = p.Results;

if o.Merge > 0
    C = merge_parts(o);
    plot_catch(C);
    return
end

cfg = harness_config();
P = struct('stab', 3, 'sup', struct('catch_th1', pi/2, 'catch_w1', 100, 'exit_th1', pi/2));
for f = fieldnames(o.Params)', P.(f{1}) = o.Params.(f{1}); end
[TH, WW] = meshgrid(o.Th1, o.W1);
N = numel(TH);
idx = o.Part(1):o.Part(2):N;
C = struct('th1', o.Th1, 'w1', o.W1, 'T', o.T, 'idx', idx, 'done', false(size(idx)), ...
    'held', nan(size(idx)), 'ipk', nan(size(idx)), 'th0max', nan(size(idx)), ...
    'upk', nan(size(idx)), 'fault', nan(size(idx)), 'th1end', nan(size(idx)));
file = part_file(o.File, o.Part);
if isfile(file)                                    % resume
    S = load(file); if isequal(S.C.idx, idx), C = S.C; end
end
w = warning('off', 'all'); cl = onCleanup(@() warning(w));
for j = find(~C.done)
    sc = furuta_scenario('balance', cfg);
    sc.name = 'catch';
    sc.T = o.T;
    sc.x0 = [0; deg2rad(TH(idx(j))); 0; WW(idx(j))];
    sc.t_event = 0;
    sc.check = @(R) R.fault == 0 && max(abs(R.th1(R.t >= o.T - 1))) < deg2rad(10);
    sc.text = 'catch test';
    R = run_sim('ctrl_hybrid.slx', sc, 'Params', P, 'Cfg', cfg);
    C.held(j) = R.pass;
    C.ipk(j) = max(abs(R.i));
    C.th0max(j) = rad2deg(max(abs(R.th0)));
    C.upk(j) = max(abs(R.u));
    C.fault(j) = R.fault;
    C.th1end(j) = rad2deg(max(abs(R.th1(R.t >= o.T - 1))));
    C.done(j) = true;
    if mod(nnz(C.done), 10) == 0 || j == numel(idx), save(file, 'C'); end
end
save(file, 'C');
fprintf('part %d/%d: %d points, %d held\n', o.Part, numel(idx), nnz(C.held == 1));
end

function f = part_file(file, part)
[d, n] = fileparts(file);
f = fullfile(d, sprintf('%s_part%d_of_%d.mat', n, part(1), part(2)));
end

function C = merge_parts(o)
n = o.Merge;
[TH, ~] = meshgrid(o.Th1, o.W1);
fields = {'held', 'ipk', 'th0max', 'upk', 'fault', 'th1end'};
for f = fields, C.(f{1}) = nan(size(TH)); end
for k = 1:n
    S = load(part_file(o.File, [k n]));
    for f = fields, C.(f{1})(S.C.idx(S.C.done)) = S.C.(f{1})(S.C.done); end
end
% mirror: (theta1, w1) -> (-theta1, -w1)
C.th1 = [-fliplr(o.Th1(o.Th1 > 0)), o.Th1];
C.w1 = o.W1;
assert(isequal(o.W1, -fliplr(o.W1)), 'W1 must be symmetric about 0 for the mirror');
for f = fields
    Z = C.(f{1});
    M = flipud(fliplr(Z(:, o.Th1 > 0)));
    C.(f{1}) = [M, Z];
end
C.held = double(C.fault == 0 & C.th1end < 10);   % same criterion as the runs
C.T = o.T;
save(o.File, 'C');
fprintf('merged -> %s\n', o.File);
end

function plot_catch(C)
fig = figure('Name', 'LQR catch region', 'Color', 'w');
try, fig.Theme = 'light'; catch, end
held = C.held == 1;
imagesc(C.th1, C.w1, C.ipk .* (held ./ held)); axis xy; hold on
colormap(parula); cb = colorbar; cb.Label.String = 'peak current [A] (held points)';
[I, J] = find(C.fault > 0); plot(C.th1(J), C.w1(I), 'rx', 'MarkerSize', 6, 'LineWidth', 1.2);
[I, J] = find(~held & ~(C.fault > 0)); plot(C.th1(J), C.w1(I), 'k.', 'MarkerSize', 8);
xlabel('\theta_1 at hand-over [deg]'); ylabel('\omega_1 at hand-over [rad/s]');
title('LQR catch region (arm centred, at rest).  x = firmware trip, . = falls');
end
