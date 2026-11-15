function results = run_tests()
%RUN_TESTS Offline-only backend, protocol, simulator and GUI construction tests.
root=fileparts(mfilename('fullpath'));addpath(root);
results=runtests(fullfile(root,'tests'));
disp(table(results));assert(all([results.Passed]),'Furuta:Tests','Some workbench tests failed');
end
