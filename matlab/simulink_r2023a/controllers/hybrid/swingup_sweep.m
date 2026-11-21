function S = swingup_sweep(knob1, vals1, varargin)
%SWINGUP_SWEEP Swing-up runs over one knob, or over a grid of two knobs.
%   S = swingup_sweep('a_push', 30:10:90)                              one knob, others from the design file
%   S = swingup_sweep('a_push', 30:10:90, 'Knob2', 'wc', 'Vals2', 2:6) 2D grid
%   Options: 'Part' [k n] (every n-th point from k, for parallel MATLABs), 'File' result name
%            (default swingup_sweep_<knobs>.mat next to this file), 'Cfg', 'Plant', 'Extra'
%            (other controller knobs), 'Merge' n (merge n part files and return the result).
%   Each point: the 'swingup' scenario, realistic sensors and firmware. Saved after every run.
%   Result fields: pass, tcap (time to catch [s]), ipk (peak current [A]), th0 (max |theta0| [deg]),
%   fault (firmware code), each size [numel(Vals2) numel(vals1)].
p = inputParser;
p.addParameter('Knob2', '');
p.addParameter('Vals2', NaN);
p.addParameter('Part', [1 1]);
p.addParameter('File', '');
p.addParameter('Cfg', struct());
p.addParameter('Plant', struct());
p.addParameter('Extra', struct());
p.addParameter('Merge', 0);
p.parse(varargin{:});
o = p.Results;
if isempty(o.File)
    name = ['swingup_sweep_' knob1];
    if ~isempty(o.Knob2), name = [name '_' o.Knob2]; end
    o.File = fullfile(fileparts(mfilename('fullpath')), [name '.mat']);
end
[V1, V2] = meshgrid(vals1, o.Vals2);
fields = {'pass', 'tcap', 'ipk', 'th0', 'fault'};
if o.Merge > 0
    S = struct('knob1', knob1, 'vals1', vals1, 'knob2', o.Knob2, 'vals2', o.Vals2);
    for f = fields, S.(f{1}) = nan(size(V1)); end
    for k = 1:o.Merge
        Q = load(part_file(o.File, [k o.Merge]));
        for f = fields, S.(f{1})(Q.S.idx(Q.S.done)) = Q.S.(f{1})(Q.S.done); end
    end
    save(o.File, 'S');
    return
end
cfg = o.Cfg; if isempty(fieldnames(cfg)), cfg = harness_config(); end
idx = o.Part(1):o.Part(2):numel(V1);
S = struct('knob1', knob1, 'vals1', vals1, 'knob2', o.Knob2, 'vals2', o.Vals2, 'idx', idx, ...
    'done', false(size(idx)));
for f = fields, S.(f{1}) = nan(size(idx)); end
file = part_file(o.File, o.Part);
if isfile(file), Q = load(file); if isequal(Q.S.idx, idx), S = Q.S; end, end
w = warning('off', 'all'); cl = onCleanup(@() warning(w));
for j = find(~S.done)
    P = o.Extra;
    P.swing.(knob1) = V1(idx(j));
    if ~isempty(o.Knob2), P.swing.(o.Knob2) = V2(idx(j)); end
    R = run_sim('ctrl_hybrid.slx', 'swingup', 'Params', P, 'Cfg', cfg, 'Plant', o.Plant);
    S.pass(j) = R.pass;
    S.tcap(j) = R.metrics.t_capture;
    S.ipk(j) = max(abs(R.i));
    S.th0(j) = rad2deg(max(abs(R.th0)));
    S.fault(j) = R.fault;
    S.done(j) = true;
    save(file, 'S');
end
if isequal(o.Part, [1 1])
    S = swingup_sweep(knob1, vals1, 'Knob2', o.Knob2, 'Vals2', o.Vals2, 'File', o.File, 'Merge', 1);
end
end

function f = part_file(file, part)
[d, n] = fileparts(file);
f = fullfile(d, sprintf('%s_part%d_of_%d.mat', n, part(1), part(2)));
end
