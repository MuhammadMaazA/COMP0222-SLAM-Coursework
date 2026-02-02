% Q1b: Test the PlatformPredictionEdge implementation
% This script tests the GPS-enabled localization system with the
% factor graph prediction step.

import ebe.core.*;
import ebe.graphics.*;

% Clear previous figures
close all;

% Load configuration
config = ebe.utils.readJSONFile('config/q1_b.json');

% Create the mainloop
mainLoop = ebe.MainLoop(config);

% Create the simulator
simulator = cw1s.drivebot.Simulator(config);
mainLoop.setEventGenerator(simulator);

% Create the G2O SLAM system
g2oSLAM = cw1s.drivebot.G2OSLAMSystem(config);
mainLoop.addEstimator(g2oSLAM);

% Create the results accumulator
resultsAccumulator = ebe.slam.XPPlatformAccumulator();
mainLoop.addResultsAccumulator(resultsAccumulator);

% Set up visualization
fig = FigureManager.getFigure("Q1b - Scenario Output");
clf
hold on
axis([-30 30 -30 30])
axis square

% Run the main loop
mainLoop.run();

% Run final optimization
g2oSLAM.optimize(10);

% Get the results
THistory = resultsAccumulator.timeStore;
XTrueHistory = resultsAccumulator.xTrueStore;
XEstHistory = resultsAccumulator.xEstStore;
PEstHistory = resultsAccumulator.PEstStore;

% Plot the trajectory
figure('Name', 'Q1b - Trajectory');
clf
hold on
plot(XTrueHistory(1,:), XTrueHistory(2,:), 'g-', 'LineWidth', 2, 'DisplayName', 'True');
plot(XEstHistory{1}(1,:), XEstHistory{1}(2,:), 'b--', 'LineWidth', 2, 'DisplayName', 'G2O Estimate');
xlabel('x (m)');
ylabel('y (m)');
legend('Location', 'best');
title('Vehicle Trajectory');
axis equal
grid on

% Plot position errors
figure('Name', 'Q1b - Position Errors');
clf

subplot(3,1,1);
hold on
xErr = XEstHistory{1}(1,:) - XTrueHistory(1,:);
xStd = sqrt(PEstHistory{1}(1,:));
plot(THistory, xErr, 'b-', 'LineWidth', 1);
plot(THistory, 2*xStd, 'r--', 'LineWidth', 1);
plot(THistory, -2*xStd, 'r--', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('x error (m)');
title('X Position Error with 2\sigma Bounds');
legend('Error', '2\sigma bounds', 'Location', 'best');
grid on

subplot(3,1,2);
hold on
yErr = XEstHistory{1}(2,:) - XTrueHistory(2,:);
yStd = sqrt(PEstHistory{1}(2,:));
plot(THistory, yErr, 'b-', 'LineWidth', 1);
plot(THistory, 2*yStd, 'r--', 'LineWidth', 1);
plot(THistory, -2*yStd, 'r--', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('y error (m)');
title('Y Position Error with 2\sigma Bounds');
legend('Error', '2\sigma bounds', 'Location', 'best');
grid on

subplot(3,1,3);
hold on
psiErr = XEstHistory{1}(3,:) - XTrueHistory(3,:);
% Wrap heading error
psiErr = atan2(sin(psiErr), cos(psiErr));
psiStd = sqrt(PEstHistory{1}(3,:));
plot(THistory, rad2deg(psiErr), 'b-', 'LineWidth', 1);
plot(THistory, rad2deg(2*psiStd), 'r--', 'LineWidth', 1);
plot(THistory, rad2deg(-2*psiStd), 'r--', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('\psi error (deg)');
title('Heading Error with 2\sigma Bounds');
legend('Error', '2\sigma bounds', 'Location', 'best');
grid on

disp('Q1b test completed.');
disp('Check if errors stay within 2-sigma bounds to verify consistency.');
