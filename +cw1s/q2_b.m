% Q2b: Test the LandmarkRangeBearingEdge implementation
% This script tests the full SLAM system with landmark observations.
% It compares G2O with EKF-SLAM when noise is disabled.

import ebe.core.*;
import ebe.graphics.*;

% Clear previous figures
close all;

% Load configuration
config = ebe.utils.readJSONFile('config/q2_b.json');

% Create the mainloop
mainLoop = ebe.MainLoop(config);

% Create the simulator
simulator = cw1s.drivebot.Simulator(config);
mainLoop.setEventGenerator(simulator);

% Create the EKF SLAM system
ekfSLAM = cw1s.drivebot.EKFSLAMSystem(config);
mainLoop.addEstimator(ekfSLAM);

% Create the G2O SLAM system
g2oSLAM = cw1s.drivebot.G2OSLAMSystem(config);
mainLoop.addEstimator(g2oSLAM);

% Create the results accumulator
resultsAccumulator = ebe.slam.XPPlatformAccumulator();
mainLoop.addResultsAccumulator(resultsAccumulator);

% Set up visualization
fig = FigureManager.getFigure("Q2b - Scenario Output");
clf
hold on
axis([-40 40 -40 40])
axis square

% Run the main loop
mainLoop.run();

% Run final optimization for G2O
g2oSLAM.optimize(10);

% Get results
THistory = resultsAccumulator.timeStore;
XTrueHistory = resultsAccumulator.xTrueStore;

% Get EKF estimates
[xEKF, PEKF] = ekfSLAM.platformEstimate();
[mEKF, PmmEKF, landmarkIdsEKF] = ekfSLAM.landmarkEstimates();

% Get G2O estimates
[xG2O, PG2O] = g2oSLAM.platformEstimate();
[mG2O, PmmG2O, landmarkIdsG2O] = g2oSLAM.landmarkEstimates();

% Plot the results
figure('Name', 'Q2b - SLAM Comparison');
clf
hold on

% Plot landmarks
landmarks = simulator.landmarks;
plot(landmarks(1,:), landmarks(2,:), 'g+', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'True Landmarks');

% Plot EKF landmark estimates
if ~isempty(mEKF)
    plot(mEKF(1,:), mEKF(2,:), 'mo', 'MarkerSize', 8, 'LineWidth', 1.5, 'DisplayName', 'EKF Landmarks');
end

% Plot G2O landmark estimates
if ~isempty(mG2O)
    plot(mG2O(1,:), mG2O(2,:), 'b^', 'MarkerSize', 8, 'LineWidth', 1.5, 'DisplayName', 'G2O Landmarks');
end

% Plot trajectory
plot(XTrueHistory(1,:), XTrueHistory(2,:), 'k-', 'LineWidth', 1.5, 'DisplayName', 'True Path');

xlabel('x (m)');
ylabel('y (m)');
legend('Location', 'best');
title('Q2b - SLAM System Comparison (No Noise)');
axis equal
grid on

% Compute landmark estimation errors
if ~isempty(mEKF) && size(landmarks,2) > 0
    % For each estimated landmark, find the closest true landmark
    ekfErrors = [];
    g2oErrors = [];
    
    for l = 1:size(mEKF, 2)
        dists = sqrt(sum((landmarks - mEKF(:,l)).^2, 1));
        ekfErrors(l) = min(dists);
    end
    
    for l = 1:size(mG2O, 2)
        dists = sqrt(sum((landmarks - mG2O(:,l)).^2, 1));
        g2oErrors(l) = min(dists);
    end
    
    fprintf('\nQ2b Landmark Estimation Errors:\n');
    fprintf('EKF - Mean: %.4f m, Max: %.4f m\n', mean(ekfErrors), max(ekfErrors));
    fprintf('G2O - Mean: %.4f m, Max: %.4f m\n', mean(g2oErrors), max(g2oErrors));
end

% Get performance data
[chi2Store, optTimeStore] = g2oSLAM.getPerformanceData();

if ~isempty(chi2Store)
    % Plot chi2 and timing
    figure('Name', 'Q2b - G2O Performance');
    
    subplot(2,1,1);
    plot(chi2Store, 'b-', 'LineWidth', 1);
    xlabel('Step');
    ylabel('Chi2');
    title('Chi2 Values');
    grid on
    
    subplot(2,1,2);
    plot(optTimeStore * 1000, 'r-', 'LineWidth', 1);
    xlabel('Step');
    ylabel('Time (ms)');
    title('Optimization Time');
    grid on
end

disp(' ');
disp('Q2b test completed.');
disp('When noise is disabled (perturbWithNoise=false), both systems should give identical results.');
