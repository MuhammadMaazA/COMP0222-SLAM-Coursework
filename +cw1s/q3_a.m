% Q3a: Graph Pruning Strategy
% This script compares the original G2O system with one that limits
% the maximum number of observations per landmark.

import ebe.core.*;
import ebe.graphics.*;

% Clear previous figures
close all;

% Load configuration
config = ebe.utils.readJSONFile('config/q3_a.json');

% Create the mainloop
mainLoop = ebe.MainLoop(config);

% Create the simulator
simulator = cw1s.drivebot.Simulator(config);
mainLoop.setEventGenerator(simulator);

% Create the EKF SLAM system
ekfSLAM = cw1s.drivebot.EKFSLAMSystem(config);
mainLoop.addEstimator(ekfSLAM);

% Create the original G2O SLAM system (no pruning)
g2oSLAMOriginal = cw1s.drivebot.G2OSLAMSystem(config);
g2oSLAMOriginal.setOptimizeOnStep(true);
g2oSLAMOriginal.setMaxObservationsPerLandmark(inf);  % No limit
mainLoop.addEstimator(g2oSLAMOriginal);

% Create the pruned G2O SLAM system
g2oSLAMPruned = cw1s.drivebot.G2OSLAMSystem(config);
g2oSLAMPruned.setOptimizeOnStep(true);
g2oSLAMPruned.setMaxObservationsPerLandmark(4);  % Max 4 observations per landmark
mainLoop.addEstimator(g2oSLAMPruned);

% Create the results accumulator
resultsAccumulator = ebe.slam.XPPlatformAccumulator();
mainLoop.addResultsAccumulator(resultsAccumulator);

% Set up visualization
fig = FigureManager.getFigure("Q3a - Scenario Output");
clf
hold on
axis([-60 60 -60 60])
axis square

% Run the main loop
mainLoop.run();

% Run final optimizations
g2oSLAMOriginal.optimize(10);
g2oSLAMPruned.optimize(10);

% Get results
THistory = resultsAccumulator.timeStore;
XTrueHistory = resultsAccumulator.xTrueStore;

% Get performance data
[chi2Original, optTimeOriginal] = g2oSLAMOriginal.getPerformanceData();
[chi2Pruned, optTimePruned] = g2oSLAMPruned.getPerformanceData();

% Plot the map comparison
figure('Name', 'Q3a - Map Comparison');
clf

% Get landmark estimates
[mEKF, ~, ~] = ekfSLAM.landmarkEstimates();
[mOriginal, ~, ~] = g2oSLAMOriginal.landmarkEstimates();
[mPruned, ~, ~] = g2oSLAMPruned.landmarkEstimates();

landmarks = simulator.landmarks;

hold on
plot(landmarks(1,:), landmarks(2,:), 'g+', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'True');
if ~isempty(mEKF)
    plot(mEKF(1,:), mEKF(2,:), 'mo', 'MarkerSize', 6, 'DisplayName', 'EKF');
end
if ~isempty(mOriginal)
    plot(mOriginal(1,:), mOriginal(2,:), 'b^', 'MarkerSize', 6, 'DisplayName', 'G2O Original');
end
if ~isempty(mPruned)
    plot(mPruned(1,:), mPruned(2,:), 'rs', 'MarkerSize', 6, 'DisplayName', 'G2O Pruned');
end
plot(XTrueHistory(1,:), XTrueHistory(2,:), 'k-', 'LineWidth', 1);
xlabel('x (m)');
ylabel('y (m)');
legend('Location', 'best');
title('Q3a - Map with Pruning Strategy');
axis equal
grid on

% Plot chi2 comparison
figure('Name', 'Q3a - Chi2 Comparison');
clf
hold on
if ~isempty(chi2Original)
    plot(chi2Original, 'b-', 'LineWidth', 1, 'DisplayName', 'Original');
end
if ~isempty(chi2Pruned)
    plot(chi2Pruned, 'r-', 'LineWidth', 1, 'DisplayName', 'Pruned (max 4 obs)');
end
xlabel('Optimization Step');
ylabel('Chi2 Value');
title('Chi2 Values - Original vs Pruned');
legend('Location', 'best');
grid on

% Plot optimization time comparison
figure('Name', 'Q3a - Optimization Time Comparison');
clf
hold on
if ~isempty(optTimeOriginal)
    plot(optTimeOriginal * 1000, 'b-', 'LineWidth', 1, 'DisplayName', 'Original');
end
if ~isempty(optTimePruned)
    plot(optTimePruned * 1000, 'r-', 'LineWidth', 1, 'DisplayName', 'Pruned (max 4 obs)');
end
xlabel('Optimization Step');
ylabel('Optimization Time (ms)');
title('Optimization Time - Original vs Pruned');
legend('Location', 'best');
grid on

% Plot error comparison
figure('Name', 'Q3a - Position Error Comparison');
XEstEKF = resultsAccumulator.xEstStore{1};
XEstOriginal = resultsAccumulator.xEstStore{2};
XEstPruned = resultsAccumulator.xEstStore{3};

PEstEKF = resultsAccumulator.PEstStore{1};
PEstOriginal = resultsAccumulator.PEstStore{2};
PEstPruned = resultsAccumulator.PEstStore{3};

subplot(3,1,1);
hold on
ekfErr = sqrt((XEstEKF(1,:) - XTrueHistory(1,:)).^2 + (XEstEKF(2,:) - XTrueHistory(2,:)).^2);
ekfStd = sqrt(PEstEKF(1,:) + PEstEKF(2,:));
plot(THistory, ekfErr, 'm-', 'LineWidth', 1);
plot(THistory, 2*ekfStd, 'm--', 'LineWidth', 0.5);
xlabel('Time (s)');
ylabel('Error (m)');
title('EKF Position Error');
grid on

subplot(3,1,2);
hold on
origErr = sqrt((XEstOriginal(1,:) - XTrueHistory(1,:)).^2 + (XEstOriginal(2,:) - XTrueHistory(2,:)).^2);
origStd = sqrt(PEstOriginal(1,:) + PEstOriginal(2,:));
plot(THistory, origErr, 'b-', 'LineWidth', 1);
plot(THistory, 2*origStd, 'b--', 'LineWidth', 0.5);
xlabel('Time (s)');
ylabel('Error (m)');
title('G2O Original Position Error');
grid on

subplot(3,1,3);
hold on
prunedErr = sqrt((XEstPruned(1,:) - XTrueHistory(1,:)).^2 + (XEstPruned(2,:) - XTrueHistory(2,:)).^2);
prunedStd = sqrt(PEstPruned(1,:) + PEstPruned(2,:));
plot(THistory, prunedErr, 'r-', 'LineWidth', 1);
plot(THistory, 2*prunedStd, 'r--', 'LineWidth', 0.5);
xlabel('Time (s)');
ylabel('Error (m)');
title('G2O Pruned Position Error');
grid on

% Print summary
fprintf('\nQ3a Summary - Pruning Strategy:\n');
fprintf('====================================\n');

% Graph structure analysis
graphOriginal = g2oSLAMOriginal.getGraph();
graphPruned = g2oSLAMPruned.getGraph();

fprintf('Original G2O: %d vertices, %d edges\n', ...
    length(graphOriginal.vertices()), length(graphOriginal.edges()));
fprintf('Pruned G2O: %d vertices, %d edges\n', ...
    length(graphPruned.vertices()), length(graphPruned.edges()));

if ~isempty(optTimeOriginal)
    fprintf('\nOptimization time (mean):\n');
    fprintf('  Original: %.2f ms\n', mean(optTimeOriginal) * 1000);
    fprintf('  Pruned: %.2f ms\n', mean(optTimePruned) * 1000);
    fprintf('  Speed improvement: %.1f%%\n', (1 - mean(optTimePruned)/mean(optTimeOriginal)) * 100);
end

if ~isempty(chi2Original)
    fprintf('\nFinal chi2:\n');
    fprintf('  Original: %.4f\n', chi2Original(end));
    fprintf('  Pruned: %.4f\n', chi2Pruned(end));
end

disp(' ');
disp('Q3a test completed.');
