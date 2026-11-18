function main(mode)
%MAIN Thin public entry: run_project(mode) with paths resolved from here.
%   main('full')  reproduces every figure/table of the report (~1.5-2 h)
%   main('quick') build + design + nominal/wind sanity runs (~10 min)
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'matlab'));
run_project(mode);
end
