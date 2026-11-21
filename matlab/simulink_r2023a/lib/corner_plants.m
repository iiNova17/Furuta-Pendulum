function C = corner_plants(p, k)
%CORNER_PLANTS Physically consistent worst-case plants: the corners of what the data allows.
%   C = corner_plants(p, k)   p = identified parameters (cfg.p), k = how many standard errors (e.g. 2)
%   Returns a struct array with fields name, plant (a struct to pass as run_sim(..., 'Plant', plant)).
%
%   Why not move every parameter on its own (perturb_params): the estimation measures the RATIOS
%   Jp/G (free-swing frequency, 0.2 %), h/G (0.5 %), b1/G and tp/G very precisely; only the overall
%   scale G is uncertain (9.3 %). So a real wrong plant scales G, Jp, h, b1, tp together. The other
%   sizeable unknown is the arm's viscous friction Beq (6.6 %). Ja, Rm, Kb, tcp, tcn are known to
%   ~1 % and stay nominal. Relative standard errors: data/estimation/model_20261115/export/parameters.json (Table 6);
%   since 2026-11-20 they are applied to the default 20 Nov model as relative errors (its own least-squares errors are
%   in data/estimation/suite_20261120/export/parameters.json, before refinement).
sG = 0.002686445554398892/0.028987306162671284;        % relative standard error of G
sB = 0.0010673827536937398/0.016122253078409365;       % relative standard error of Beq
C = struct('name', {}, 'plant', {});
for g = [-1 1]
    for b = [-1 1]
        fg = 1 + g*k*sG;  fb = 1 + b*k*sB;
        q = struct('G', p.G*fg, 'Jp', p.Jp*fg, 'h', p.h*fg, 'b1', p.b1*fg, 'tp', p.tp*fg, ...
                   'Beq', p.Beq*fb);
        C(end+1) = struct('name', sprintf('pendulum %+.0f%%, arm friction %+.0f%%', 100*(fg - 1), ...
            100*(fb - 1)), 'plant', q); %#ok<AGROW>
    end
end
end
