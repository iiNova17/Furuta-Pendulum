% Exports the three rig bodies for the Unreal twin (twin/RIPTwin), from the harness's meshes
% (matlab/harness/plant3d) and the CAD frames in twin_frames.mat (from the SolidWorks export).
% Every mesh is written in the assembly's zero pose (q_arm = 0, q_pend = 0 hanging), in Unreal's frame:
%   UE = (x, -y, z) of the SolidWorks/Simscape world (right-handed -> left-handed), 1 mm -> 1 UE unit
%   (so the twin is a 10x scale model, which keeps it clear of Unreal's 10-unit near clip plane).
% File format (.ripmesh, little-endian): uint32 'RIPM', uint32 nTri, then nTri x 3 vertices of float32 xyz,
% unshared so the twin can shade every CAD face flat.  Also writes twin_geometry.json (pivot + axes).
here = fileparts(mfilename('fullpath'));
sim = fullfile(here, '..', '..', 'matlab', 'harness', 'plant3d');
out = fullfile(here, '..', 'RIPTwin', 'Content', 'RIP');
if ~exist(out, 'dir'), mkdir(out); end
S = load(fullfile(here, 'twin_frames.mat')); fr = S.P.frames;

ue = @(p) [p(:, 1), -p(:, 2), p(:, 3)];                         % world (mm) -> Unreal
writeMesh(fullfile(out, 'housing.ripmesh'), stlread(fullfile(sim, 'body_base_elec8.stl')), @(p) p, ue);
writeMesh(fullfile(out, 'arm.ripmesh'), stlread(fullfile(sim, 'body_arm_jumper3.stl')), ...
    @(p) (fr.arm.R * p' + fr.arm.t)', ue);
pend2arm = @(p) (fr.piv.t + fr.piv.R * (fr.pend.t + fr.pend.R * p'))';
writeMesh(fullfile(out, 'pendulum.ripmesh'), stlread(fullfile(sim, 'body_pendulum_egp.stl')), ...
    @(p) (fr.arm.R * pend2arm(p)' + fr.arm.t)', ue);

% joint axes in Unreal: a right-handed rotation by a about n is a rotation by -a about (nx, -ny, nz)
pivot = fr.arm.R * fr.piv.t + fr.arm.t;
pend_axis = fr.arm.R * fr.piv.R * [0; 0; 1];
geo.units = 'mm (= Unreal units)';
geo.arm_axis = [0 0 1];
geo.pivot = ue(pivot');
geo.pend_axis = ue(pend_axis');
geo.note = 'Rotate by -angle about these axes (handedness flip). q_pend = 0 hanging, pi upright.';
fid = fopen(fullfile(out, 'twin_geometry.json'), 'w'); fprintf(fid, '%s\n', jsonencode(geo, 'PrettyPrint', true)); fclose(fid);
fprintf('pivot (UE) = [%s], pendulum axis (UE) = [%s]\n', num2str(geo.pivot, 5), num2str(geo.pend_axis, 5));

function writeMesh(file, tr, toWorld, ue)
    V = ue(toWorld(tr.Points));
    F = tr.ConnectivityList(:, [1 3 2]);                        % mirroring flips the winding back
    P = V(F', :)';                                              % 3 x (3*nTri), unshared vertices
    fid = fopen(file, 'w', 'ieee-le');
    fwrite(fid, uint8('RIPM'), 'uint8'); fwrite(fid, size(F, 1), 'uint32'); fwrite(fid, single(P(:)), 'single');
    fclose(fid);
    fprintf('%s: %d triangles, z %.1f..%.1f mm\n', file, size(F, 1), min(V(:, 3)), max(V(:, 3)));
end
