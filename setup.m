% Setup script for COMP0222 Coursework 1 - Factor Graph SLAM
% Run this script before running any coursework questions

fprintf('Setting up COMP0222 Coursework 1...\n');

% Get the path to this script
cwPath = fileparts(mfilename('fullpath'));

% Add paths
addpath(cwPath);
addpath(fullfile(cwPath, 'Libraries'));
addpath(fullfile(cwPath, 'Labs', 'Lab3_EKF_SLAM', 'Code'));

fprintf('Setup complete.\n\n');
fprintf('Available scripts:\n');
fprintf('  >> cw1s.q1_b   - Q1b: Test PlatformPredictionEdge\n');
fprintf('  >> cw1s.q1_c   - Q1c: Chi2 and timing analysis\n');
fprintf('  >> cw1s.q2_b   - Q2b: Test LandmarkRangeBearingEdge\n');
fprintf('  >> cw1s.q2_c   - Q2c: Large scenario comparison\n');
fprintf('  >> cw1s.q3_a   - Q3a: Graph pruning strategy\n');
fprintf('  >> cw1s.q3_b   - Q3b: Vertex fixing strategy\n');
