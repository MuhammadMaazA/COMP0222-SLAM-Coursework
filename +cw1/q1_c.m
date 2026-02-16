% Q1c: Analyse chi2 and optimisation time behaviour

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

