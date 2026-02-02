% Q1c: Analyze graph optimization behavior over time
% This script optimizes the graph at every time step and records
% chi2 values and optimization times.

import ebe.core.*;
import ebe.graphics.*;

% Clear previous figures
close all;

% Load configuration
config = ebe.utils.readJSONFile('config/q1_c.json');

% Create the mainloop
mainLoop = ebe.MainLoop(config);

% Create the simulator
simulator = cw1s.drivebot.Simulator(config);
mainLoop.setEventGenerator(simulator);

% Create the G2O SLAM system with optimization on each step
g2oSLAM = cw1s.drivebot.G2OSLAMSystem(config);
g2oSLAM.setOptimizeOnStep(true);
mainLoop.addEstimator(g2oSLAM);

% Create the results accumulator
resultsAccumulator = ebe.slam.XPPlatformAccumulator();
mainLoop.addResultsAccumulator(resultsAccumulator);

% Set up visualization
fig = FigureManager.getFigure("Q1c - Scenario Output");
clf
hold on
axis([-30 30 -30 30])
axis square

% Run the main loop
mainLoop.run();

% Get performance data
[chi2Store, optTimeStore] = g2oSLAM.getPerformanceData();

% Plot chi2 values over time
figure('Name', 'Q1c - Chi2 Values');
clf
plot(chi2Store, 'b-', 'LineWidth', 1);
xlabel('Optimization Step');
ylabel('chi2 Value');
title('Chi2 Values Over Time');
grid on

% Plot optimization times over time
figure('Name', 'Q1c - Optimization Times');
clf
plot(optTimeStore * 1000, 'r-', 'LineWidth', 1);
xlabel('Optimization Step');
ylabel('Optimization Time (ms)');
title('Optimization Time Over Time');
grid on

% Print summary statistics
fprintf('\nQ1c Performance Summary:\n');
fprintf('Total optimizations: %d\n', length(chi2Store));
fprintf('Mean chi2: %.4f\n', mean(chi2Store));
fprintf('Final chi2: %.4f\n', chi2Store(end));
fprintf('Mean optimization time: %.2f ms\n', mean(optTimeStore) * 1000);
fprintf('Total optimization time: %.2f s\n', sum(optTimeStore));

disp(' ');
disp('Q1c analysis completed.');
disp('Observe the trends in chi2 values and optimization times.');
