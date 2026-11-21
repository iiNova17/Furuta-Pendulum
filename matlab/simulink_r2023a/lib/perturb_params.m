function p = perturb_params(p, k, seed)
%PERTURB_PARAMS Draw plant parameters within the identification uncertainty (report Table 6).
%   p = perturb_params(p, k, seed): each parameter is multiplied by (1 + k*sigma*n), n ~ N(0,1),
%   with sigma its relative standard error from Table 6. k = 1 is "as uncertain as the report says";
%   k = 2 or 3 is a harsher robustness test.
sigma = struct('Rm', 0.004, 'Kb', 0.003, 'Ja', 0.004, 'Jp', 0.09, 'h', 0.08, 'G', 0.09, ...
               'Beq', 0.07, 'b1', 0.11, 'tcp', 0.013, 'tcn', 0.013, 'tp', 0.08);
rs = RandStream('mt19937ar', 'Seed', seed);
f = fieldnames(sigma);
for j = 1:numel(f)
    p.(f{j}) = p.(f{j})*max(0.05, 1 + k*sigma.(f{j})*randn(rs));
end
end
