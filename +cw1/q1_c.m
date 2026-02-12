% Q1c: Analyse chi2 and optimisation time behaviour
%
% This script runs the GPS-enabled localisation system, optimising the
% graph at every time step (which is very inefficient). Once finished,
% it plots chi2 values and optimisation durations to study how they
% evolve as the graph grows.
%
% Key observations:
%   1. Chi2 increases in a staircase pattern (GPS events)
%   2. Optimisation time also grows (larger Hessian)
%   3. Both are driven by the monotonically growing graph size

import ebe.core.*;
import ebe.graphics.*;
import cw1.*;

% Find, load and parse the configuration file
config = ebe.utils.readJSONFile('config/q1_c.json');

% Create the mainloop object
mainLoop = ebe.MainLoop(config);

% Create the simulator and register it
simulator = drivebot.Simulator(config);
mainLoop.setEventGenerator(simulator);

% Create the SLAM system and register it
g2oSLAMSystem = drivebot.G2OSLAMSystem(config);
mainLoop.addEstimator(g2oSLAMSystem);

% Create the results accumulator (every time step for Q1c)
resultsAccumulator = ebe.slam.XPPlatformAccumulator();
mainLoop.addResultsAccumulator(resultsAccumulator);
mainLoop.setAccumulateResultsUpdatePeriod(1);

% Set up the live simulation figure
fig = FigureManager.getFigure('Q1c: Live Simulation');
clf
hold on

% Compute axis bounds from waypoints (no hardcoding)
wp = config.platformTrajectory.waypoints;
if iscell(wp)
    wpMat = cell2mat(wp');
else
    wpMat = wp;
end
xLo = min(wpMat(:,1));  xHi = max(wpMat(:,1));
yLo = min(wpMat(:,2));  yHi = max(wpMat(:,2));
span = max(xHi - xLo, yHi - yLo);
pad = max(span * 0.15, 5);
axis([xLo-pad, xHi+pad, yLo-pad, yHi+pad])

% Set up the views
simulatorViewer = ebe.graphics.ViewManager(config);
simulatorView = drivebot.SimulatorView(config, simulator);
simulatorViewer.addView(simulatorView);
simulatorViewer.addView(drivebot.SLAMSystemView(config, g2oSLAMSystem));

mainLoop.addViewer(simulatorViewer);
mainLoop.setGraphicsUpdatePeriod(1);

% Run the main loop
mainLoop.run();

% =====================================================================
% Extract stored data
% =====================================================================

T = resultsAccumulator.timeStore;
XTrue = resultsAccumulator.xTrueStore;
numStates = size(XTrue, 1);

stateLabels = {'$x$', '$y$', '$\psi$'};
yLabels = {'$x$ error (m)', '$y$ error (m)', '$\psi$ error (rad)'};

for e = 1 : numel(resultsAccumulator.xEstStore)

    PX = resultsAccumulator.PEstStore{e};
    XEst = resultsAccumulator.xEstStore{e};
    numTimeSteps = size(XEst, 2);

    % --- Trajectory Comparison ---
    ebe.graphics.FigureManager.getFigure('Q1c: Trajectory Comparison');
    clf
    plot(XTrue(1,:), XTrue(2,:), 'g-', 'LineWidth', 2)
    hold on
    plot(XEst(1,:), XEst(2,:), 'b--', 'LineWidth', 1.5)
    plot(XTrue(1,1), XTrue(2,1), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k')
    plot(XTrue(1,end), XTrue(2,end), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r')
    hold off
    grid on
    axis equal
    xlabel('$x$ (m)', 'Interpreter', 'latex', 'FontSize', 12)
    ylabel('$y$ (m)', 'Interpreter', 'latex', 'FontSize', 12)
    title('Q1c: True vs Estimated Trajectory', 'FontSize', 14)
    legend('Ground Truth', 'Estimated', 'Start', 'End', 'Location', 'best')
    set(gca, 'FontSize', 11)

    % --- State Estimation Errors with 2-sigma bounds ---
    ebe.graphics.FigureManager.getFigure('Q1c: State Estimation Errors');
    clf

    for f = 1 : numStates
        subplot(numStates, 1, f)
        sigmaBound = 2 * sqrt(PX(f, :));

        fill([T, fliplr(T)], [sigmaBound, fliplr(-sigmaBound)], ...
             [1 0.8 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.5)
        hold on
        plot(T, sigmaBound, 'r--', 'LineWidth', 1.5)
        plot(T, -sigmaBound, 'r--', 'LineWidth', 1.5)

        stateError = XEst(f, :) - XTrue(f, :);
        if f == 3
            stateError = mod(stateError + pi, 2*pi) - pi;
        end
        plot(T, stateError, 'b-', 'LineWidth', 1.5)

        hold off
        grid on
        maxVal = max(max(abs(stateError)), max(sigmaBound));
        bound = 1.1 * maxVal;
        axis([T(1) T(end) -bound bound])

        xlabel('Time (s)', 'FontSize', 11)
        ylabel(yLabels{f}, 'Interpreter', 'latex', 'FontSize', 12)
        title(stateLabels{f}, 'Interpreter', 'latex', 'FontSize', 13)
        if f == 1
            legend('$2\sigma$ region', '$+2\sigma$', '$-2\sigma$', 'Error', ...
                   'Interpreter', 'latex', 'Location', 'best', 'FontSize', 9)
        end
        set(gca, 'FontSize', 10)
    end
    sgtitle('Q1c: Estimation Error with $2\sigma$ Covariance Bounds', ...
            'Interpreter', 'latex', 'FontSize', 14)

    % --- Console summary ---
    fprintf('\n===== Q1c Estimation Results =====\n');
    posError = XEst(1:2, :) - XTrue(1:2, :);
    headError = mod(XEst(3,:) - XTrue(3,:) + pi, 2*pi) - pi;
    fprintf('RMSE x:       %.4f m\n', sqrt(mean(posError(1,:).^2)));
    fprintf('RMSE y:       %.4f m\n', sqrt(mean(posError(2,:).^2)));
    fprintf('RMSE heading: %.4f rad (%.2f deg)\n', ...
            sqrt(mean(headError.^2)), rad2deg(sqrt(mean(headError.^2))));
    fprintf('\n');
    allErrors = [posError(1,:); posError(2,:); headError];
    allSigma = 2 * sqrt(PX);
    for f = 1 : numStates
        pct = 100 * sum(abs(allErrors(f,:)) <= allSigma(f,:)) / numTimeSteps;
        fprintf('%s within 2-sigma: %.1f%% (expected ~95.4%%)\n', ...
                stateLabels{f}, pct);
    end
    fprintf('==================================\n\n');
end

% =====================================================================
% Q1c CORE: Chi2 and Optimisation Time Plots
% =====================================================================

g2oPerfData = g2oSLAMSystem.getPerformanceData();
chi2Values = g2oPerfData.get('g2o.op.chi2');
optimDurations = g2oPerfData.get('g2o.op.op_dt');

numChi2 = numel(chi2Values);
numDur = numel(optimDurations);

ebe.graphics.FigureManager.getFigure('Q1c: Chi2 and Optimisation Time');
clf

% Chi2 plot
subplot(2, 1, 1)
plot(1:numChi2, chi2Values, 'b-', 'LineWidth', 1.5)
grid on
xlabel('Optimisation step', 'FontSize', 11)
ylabel('$\chi^2$', 'Interpreter', 'latex', 'FontSize', 12)
title('Cost function ($\chi^2$) at each optimisation step', ...
      'Interpreter', 'latex', 'FontSize', 13)
set(gca, 'FontSize', 10)

% Optimisation duration plot
subplot(2, 1, 2)
plot(1:numDur, optimDurations, 'r-', 'LineWidth', 1.5)
grid on
xlabel('Optimisation step', 'FontSize', 11)
ylabel('Duration (s)', 'FontSize', 12)
title('Optimisation duration at each step', 'FontSize', 13)
set(gca, 'FontSize', 10)

sgtitle('Q1c: $\chi^2$ and Optimisation Time Trends', ...
        'Interpreter', 'latex', 'FontSize', 14)

% =====================================================================
% Console: Quantitative trend analysis
% =====================================================================
fprintf('===== Q1c: Chi2 and Timing Analysis =====\n');
fprintf('Total optimisation steps:  %d\n', numChi2);
fprintf('Chi2 range:                [%.2f, %.2f]\n', min(chi2Values), max(chi2Values));
fprintf('Final chi2:                %.2f\n', chi2Values(end));
fprintf('Mean optimisation time:    %.4f s\n', mean(optimDurations));
fprintf('Max optimisation time:     %.4f s\n', max(optimDurations));
fprintf('\n');

% Fit linear trend to chi2
if numChi2 > 10
    p = polyfit((1:numChi2), chi2Values, 1);
    fprintf('Chi2 linear fit:     chi2 ~ %.4f * step + %.4f\n', p(1), p(2));
end

% Fit linear trend to optimisation times
if numDur > 10
    pT = polyfit((1:numDur), optimDurations, 1);
    fprintf('Timing linear fit:   dt   ~ %.6f * step + %.6f\n', pT(1), pT(2));
end

fprintf('\n--- Observations ---\n');
fprintf('1. Chi2 shows a STAIRCASE pattern. Each step jump occurs when a\n');
fprintf('   GPS measurement is added (every %.1f s). Between GPS events,\n', ...
        config.scenario.sensors.gps.measurementPeriod);
fprintf('   chi2 stays flat because prediction edges are satisfied by\n');
fprintf('   the initial estimate (zero residual).\n');
fprintf('2. Optimisation time SPIKES coincide with GPS events. The spikes\n');
fprintf('   grow larger over time because the Hessian matrix grows with\n');
fprintf('   each new vertex/edge added to the graph.\n');
fprintf('3. UNDERLYING CAUSE: graph grows linearly in time.\n');
fprintf('   More edges => more cost terms => higher chi2.\n');
fprintf('   Larger Hessian => more expensive factorisation => longer solve.\n');
fprintf('==========================================\n\n');
