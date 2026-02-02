% Q2c: Larger scenario with smaller detection range
% This script tests both SLAM systems in a more challenging scenario.

import ebe.core.*;
import ebe.graphics.*;

% Clear previous figures
close all;

% Load configuration
config = ebe.utils.readJSONFile('config/q2_c.json');

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
fig = FigureManager.getFigure("Q2c - Scenario Output");
clf
hold on
axis([-60 60 -60 60])
axis square

% Run the main loop
mainLoop.run();

% Run final optimization for G2O
g2oSLAM.optimize(10);

% Get results
THistory = resultsAccumulator.timeStore;
XTrueHistory = resultsAccumulator.xTrueStore;

% Get landmark estimates
[mEKF, PmmEKF, landmarkIdsEKF] = ekfSLAM.landmarkEstimates();
[mG2O, PmmG2O, landmarkIdsG2O] = g2oSLAM.landmarkEstimates();

% Plot the final map
figure('Name', 'Q2c - SLAM Comparison');
clf
hold on

% Plot true landmarks
landmarks = simulator.landmarks;
plot(landmarks(1,:), landmarks(2,:), 'g+', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'True Landmarks');

% Plot EKF landmarks with covariance ellipses
if ~isempty(mEKF)
    for l = 1:size(mEKF, 2)
        % Plot estimate
        plot(mEKF(1,l), mEKF(2,l), 'mo', 'MarkerSize', 6, 'LineWidth', 1);
        
        % Plot 2-sigma covariance ellipse
        if ~isempty(PmmEKF)
            Pll = PmmEKF(:,:,l);
            [V, D] = eig(Pll);
            theta = linspace(0, 2*pi, 50);
            ellipse = 2 * V * sqrt(D) * [cos(theta); sin(theta)];
            plot(mEKF(1,l) + ellipse(1,:), mEKF(2,l) + ellipse(2,:), 'm-', 'LineWidth', 0.5);
        end
    end
end

% Plot G2O landmarks with covariance ellipses
if ~isempty(mG2O)
    for l = 1:size(mG2O, 2)
        % Plot estimate
        plot(mG2O(1,l), mG2O(2,l), 'b^', 'MarkerSize', 6, 'LineWidth', 1);
        
        % Plot 2-sigma covariance ellipse
        if ~isempty(PmmG2O)
            Pll = PmmG2O(:,:,l);
            [V, D] = eig(Pll);
            theta = linspace(0, 2*pi, 50);
            ellipse = 2 * V * sqrt(D) * [cos(theta); sin(theta)];
            plot(mG2O(1,l) + ellipse(1,:), mG2O(2,l) + ellipse(2,:), 'b-', 'LineWidth', 0.5);
        end
    end
end

% Plot trajectory
plot(XTrueHistory(1,:), XTrueHistory(2,:), 'r-', 'LineWidth', 1, 'DisplayName', 'True Path');

xlabel('x (m)');
ylabel('y (m)');
legend('True Landmarks', 'EKF (magenta)', 'G2O (blue)', 'True Path', 'Location', 'best');
title('Q2c - Large Scenario with Small Detection Range');
axis equal
grid on

% Plot error analysis
figure('Name', 'Q2c - Estimation Errors');
XEstEKF = resultsAccumulator.xEstStore{1};
XEstG2O = resultsAccumulator.xEstStore{2};

subplot(2,1,1);
hold on
ekfErr = sqrt((XEstEKF(1,:) - XTrueHistory(1,:)).^2 + (XEstEKF(2,:) - XTrueHistory(2,:)).^2);
g2oErr = sqrt((XEstG2O(1,:) - XTrueHistory(1,:)).^2 + (XEstG2O(2,:) - XTrueHistory(2,:)).^2);
plot(THistory, ekfErr, 'm-', 'LineWidth', 1, 'DisplayName', 'EKF');
plot(THistory, g2oErr, 'b-', 'LineWidth', 1, 'DisplayName', 'G2O');
xlabel('Time (s)');
ylabel('Position Error (m)');
title('Platform Position Error');
legend('Location', 'best');
grid on

subplot(2,1,2);
hold on
ekfHeadErr = abs(XEstEKF(3,:) - XTrueHistory(3,:));
ekfHeadErr = atan2(sin(ekfHeadErr), cos(ekfHeadErr));
g2oHeadErr = abs(XEstG2O(3,:) - XTrueHistory(3,:));
g2oHeadErr = atan2(sin(g2oHeadErr), cos(g2oHeadErr));
plot(THistory, rad2deg(ekfHeadErr), 'm-', 'LineWidth', 1, 'DisplayName', 'EKF');
plot(THistory, rad2deg(g2oHeadErr), 'b-', 'LineWidth', 1, 'DisplayName', 'G2O');
xlabel('Time (s)');
ylabel('Heading Error (deg)');
title('Platform Heading Error');
legend('Location', 'best');
grid on

fprintf('\nQ2c Summary:\n');
fprintf('Detection Range: %d m\n', config.scenario.sensors.slam.detectionRange);
fprintf('Number of landmarks: %d\n', size(landmarks, 2));
fprintf('EKF landmarks found: %d\n', size(mEKF, 2));
fprintf('G2O landmarks found: %d\n', size(mG2O, 2));

disp(' ');
disp('Q2c test completed.');
disp('Compare EKF vs G2O consistency with the smaller detection range.');
