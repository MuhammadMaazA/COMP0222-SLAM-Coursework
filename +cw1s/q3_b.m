% Q3b: Vertex Fixing Strategy
% This script tests the strategy of fixing old vehicle poses to reduce
% computational cost, with a final optimization that unfixes everything.

import ebe.core.*;
import ebe.graphics.*;

% Clear previous figures
close all;

% Load configuration
config = ebe.utils.readJSONFile('config/q3_b.json');

% Create the mainloop
mainLoop = ebe.MainLoop(config);

% Create the simulator
simulator = cw1s.drivebot.Simulator(config);
mainLoop.setEventGenerator(simulator);

% Create the EKF SLAM system for reference
ekfSLAM = cw1s.drivebot.EKFSLAMSystem(config);
mainLoop.addEstimator(ekfSLAM);

% Create the original G2O SLAM system (no fixing)
g2oSLAMOriginal = cw1s.drivebot.G2OSLAMSystem(config);
g2oSLAMOriginal.setOptimizeOnStep(true);
mainLoop.addEstimator(g2oSLAMOriginal);

% Create the fixed G2O SLAM system
g2oSLAMFixed = cw1s.drivebot.G2OSLAMSystem(config);
g2oSLAMFixed.setOptimizeOnStep(true);
g2oSLAMFixed.setFixOldPosesTimeWindow(5);  % Fix poses older than 5 seconds
g2oSLAMFixed.setUnfixAllAtEnd(true);  % Unfix all at the end
mainLoop.addEstimator(g2oSLAMFixed);

% Create the results accumulator
resultsAccumulator = ebe.slam.XPPlatformAccumulator();
mainLoop.addResultsAccumulator(resultsAccumulator);

% Set up visualization
fig = FigureManager.getFigure("Q3b - Scenario Output");
clf
hold on
axis([-60 60 -60 60])
axis square

% Run the main loop
fprintf('Running simulation...\n');
mainLoop.run();

% Get performance data BEFORE final optimization
[chi2Original, optTimeOriginal] = g2oSLAMOriginal.getPerformanceData();
[chi2Fixed, optTimeFixed] = g2oSLAMFixed.getPerformanceData();

fprintf('\nPerformance DURING run:\n');
if ~isempty(optTimeOriginal) && ~isempty(optTimeFixed)
    fprintf('Original mean opt time: %.2f ms\n', mean(optTimeOriginal) * 1000);
    fprintf('Fixed mean opt time: %.2f ms\n', mean(optTimeFixed) * 1000);
end

% Run final optimizations
fprintf('\nRunning final optimization (all vertices unfixed)...\n');
g2oSLAMOriginal.optimize(10);
g2oSLAMFixed.finalOptimization(10);  % This unfixes all vertices first

% Get results
THistory = resultsAccumulator.timeStore;
XTrueHistory = resultsAccumulator.xTrueStore;

% Get landmark estimates after final optimization
[mOriginal, PmmOriginal, ~] = g2oSLAMOriginal.landmarkEstimates();
[mFixed, PmmFixed, ~] = g2oSLAMFixed.landmarkEstimates();

% Plot chi2 comparison
figure('Name', 'Q3b - Chi2 Values');
clf
hold on
if ~isempty(chi2Original)
    plot(chi2Original, 'b-', 'LineWidth', 1, 'DisplayName', 'Original');
end
if ~isempty(chi2Fixed)
    plot(chi2Fixed, 'r-', 'LineWidth', 1, 'DisplayName', 'Fixed (time window)');
end
xlabel('Optimization Step');
ylabel('Chi2 Value');
title('Chi2 Values - Original vs Fixed');
legend('Location', 'best');
grid on

% Plot optimization time comparison
figure('Name', 'Q3b - Optimization Times');
clf
hold on
if ~isempty(optTimeOriginal)
    plot(optTimeOriginal * 1000, 'b-', 'LineWidth', 1, 'DisplayName', 'Original');
end
if ~isempty(optTimeFixed)
    plot(optTimeFixed * 1000, 'r-', 'LineWidth', 1, 'DisplayName', 'Fixed (time window)');
end
xlabel('Optimization Step');
ylabel('Optimization Time (ms)');
title('Optimization Time - Original vs Fixed');
legend('Location', 'best');
grid on

% Plot the final map
figure('Name', 'Q3b - Final Map');
clf
hold on

landmarks = simulator.landmarks;
plot(landmarks(1,:), landmarks(2,:), 'g+', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'True');

% Plot with covariance ellipses
if ~isempty(mOriginal)
    for l = 1:size(mOriginal, 2)
        plot(mOriginal(1,l), mOriginal(2,l), 'b^', 'MarkerSize', 6);
        if ~isempty(PmmOriginal) && all(~isnan(PmmOriginal(:,:,l)), 'all')
            Pll = PmmOriginal(:,:,l);
            [V, D] = eig(Pll);
            D = real(D);
            if all(diag(D) > 0)
                theta = linspace(0, 2*pi, 50);
                ellipse = 2 * V * sqrt(D) * [cos(theta); sin(theta)];
                plot(mOriginal(1,l) + ellipse(1,:), mOriginal(2,l) + ellipse(2,:), 'b-', 'LineWidth', 0.5);
            end
        end
    end
end

if ~isempty(mFixed)
    for l = 1:size(mFixed, 2)
        plot(mFixed(1,l), mFixed(2,l), 'rs', 'MarkerSize', 6);
        if ~isempty(PmmFixed) && all(~isnan(PmmFixed(:,:,l)), 'all')
            Pll = PmmFixed(:,:,l);
            [V, D] = eig(Pll);
            D = real(D);
            if all(diag(D) > 0)
                theta = linspace(0, 2*pi, 50);
                ellipse = 2 * V * sqrt(D) * [cos(theta); sin(theta)];
                plot(mFixed(1,l) + ellipse(1,:), mFixed(2,l) + ellipse(2,:), 'r-', 'LineWidth', 0.5);
            end
        end
    end
end

plot(XTrueHistory(1,:), XTrueHistory(2,:), 'k-', 'LineWidth', 1);
xlabel('x (m)');
ylabel('y (m)');
legend('True Landmarks', 'G2O Original', 'G2O Fixed', 'Location', 'best');
title('Q3b - Final Map After Optimization');
axis equal
grid on

% Plot position error comparison
figure('Name', 'Q3b - Position Errors');
XEstOriginal = resultsAccumulator.xEstStore{2};
XEstFixed = resultsAccumulator.xEstStore{3};
PEstOriginal = resultsAccumulator.PEstStore{2};
PEstFixed = resultsAccumulator.PEstStore{3};

subplot(2,1,1);
hold on
origErr = sqrt((XEstOriginal(1,:) - XTrueHistory(1,:)).^2 + (XEstOriginal(2,:) - XTrueHistory(2,:)).^2);
origStd = sqrt(PEstOriginal(1,:) + PEstOriginal(2,:));
plot(THistory, origErr, 'b-', 'LineWidth', 1, 'DisplayName', 'Error');
plot(THistory, 2*origStd, 'b--', 'LineWidth', 0.5, 'DisplayName', '2\sigma');
xlabel('Time (s)');
ylabel('Error (m)');
title('G2O Original Position Error');
legend('Location', 'best');
grid on

subplot(2,1,2);
hold on
fixedErr = sqrt((XEstFixed(1,:) - XTrueHistory(1,:)).^2 + (XEstFixed(2,:) - XTrueHistory(2,:)).^2);
fixedStd = sqrt(PEstFixed(1,:) + PEstFixed(2,:));
plot(THistory, fixedErr, 'r-', 'LineWidth', 1, 'DisplayName', 'Error');
plot(THistory, 2*fixedStd, 'r--', 'LineWidth', 0.5, 'DisplayName', '2\sigma');
xlabel('Time (s)');
ylabel('Error (m)');
title('G2O Fixed Position Error');
legend('Location', 'best');
grid on

% Print summary
fprintf('\n====================================\n');
fprintf('Q3b Summary - Vertex Fixing Strategy\n');
fprintf('====================================\n');

if ~isempty(optTimeOriginal) && ~isempty(optTimeFixed)
    fprintf('\nOptimization time during run (mean):\n');
    fprintf('  Original: %.2f ms\n', mean(optTimeOriginal) * 1000);
    fprintf('  Fixed: %.2f ms\n', mean(optTimeFixed) * 1000);
    fprintf('  Speed improvement: %.1f%%\n', (1 - mean(optTimeFixed)/mean(optTimeOriginal)) * 100);
end

if ~isempty(chi2Original) && ~isempty(chi2Fixed)
    fprintf('\nChi2 values:\n');
    fprintf('  Original final: %.4f\n', chi2Original(end));
    fprintf('  Fixed final: %.4f\n', chi2Fixed(end));
end

% Landmark covariance analysis
if ~isempty(PmmOriginal) && ~isempty(PmmFixed)
    origTraces = zeros(1, size(PmmOriginal, 3));
    fixedTraces = zeros(1, size(PmmFixed, 3));
    for l = 1:size(PmmOriginal, 3)
        origTraces(l) = trace(PmmOriginal(:,:,l));
    end
    for l = 1:size(PmmFixed, 3)
        fixedTraces(l) = trace(PmmFixed(:,:,l));
    end
    fprintf('\nLandmark covariance (mean trace):\n');
    fprintf('  Original: %.4f\n', mean(origTraces(~isnan(origTraces))));
    fprintf('  Fixed: %.4f\n', mean(fixedTraces(~isnan(fixedTraces))));
end

disp(' ');
disp('Q3b test completed.');
disp('Examine the covariance behavior during and after the run.');
