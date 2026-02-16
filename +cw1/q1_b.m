% Q1b: Test the PlatformPredictionEdge implementation
%
% This script tests the GPS-enabled localisation system with the
% factor graph prediction step. It creates a g2o SLAM system,
% runs the simulation, and produces:
%   - Figure 1: Live simulation view
%   - Figure 2: True vs estimated XY trajectory
%   - Figure 3: State estimation errors (x, y, psi) with 2-sigma bounds
%   - Console: RMSE statistics and estimator consistency check

import ebe.core.*;
import ebe.graphics.*;
import cw1.*;

% Find, load and parse the configuration file
config = ebe.utils.readJSONFile('config/q1_b.json');

% Create the mainloop object, which manages everything
mainLoop = ebe.MainLoop(config);

% Create the simulator and register it
simulator = drivebot.Simulator(config);
mainLoop.setEventGenerator(simulator);

% Create the SLAM system and register it
g2oSLAMSystem = drivebot.G2OSLAMSystem(config);
mainLoop.addEstimator(g2oSLAMSystem);

% Create the results accumulator to store estimates over time
resultsAccumulator = ebe.slam.XPPlatformAccumulator();
mainLoop.addResultsAccumulator(resultsAccumulator);
mainLoop.setAccumulateResultsUpdatePeriod(50);

% Set up the live simulation figure
fig = FigureManager.getFigure('Q1b: Live Simulation');
clf
hold on
axis equal

% Set up the views which show the output of the simulator
simulatorViewer = ebe.graphics.ViewManager(config);
simulatorView = drivebot.SimulatorView(config, simulator);
simulatorView.setCentreAxesOnTruth(true);
simulatorViewer.addView(simulatorView);
simulatorViewer.addView(drivebot.SLAMSystemView(config, g2oSLAMSystem));

% Register the viewer with the mainloop
mainLoop.addViewer(simulatorViewer);
mainLoop.setGraphicsUpdatePeriod(50);

% Run the main loop until it terminates
mainLoop.run();

% =====================================================================
% Post-simulation analysis and plotting
% =====================================================================

% Extract stored data from the results accumulator
T = resultsAccumulator.timeStore;
XTrue = resultsAccumulator.xTrueStore;
numStates = size(XTrue, 1);

% State labels and units (derived from state dimension, not hardcoded)
stateLabels = {'$x$', '$y$', '$\psi$'};
yLabels = {'$x$ error (m)', '$y$ error (m)', '$\psi$ error (rad)'};

for e = 1 : numel(resultsAccumulator.xEstStore)

    PX = resultsAccumulator.PEstStore{e};
    XEst = resultsAccumulator.xEstStore{e};
    numTimeSteps = size(XEst, 2);

    % -----------------------------------------------------------------
    % Figure: XY Trajectory Comparison
    % -----------------------------------------------------------------
    ebe.graphics.FigureManager.getFigure('Q1b: Trajectory Comparison');
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
    title('Q1b: True vs Estimated Trajectory', 'FontSize', 14)
    legend('Ground Truth', 'Estimated', 'Start', 'End', 'Location', 'best')
    set(gca, 'FontSize', 11)

    % -----------------------------------------------------------------
    % Figure: State Estimation Errors with 2-sigma Bounds
    % -----------------------------------------------------------------
    ebe.graphics.FigureManager.getFigure('Q1b: State Estimation Errors');
    clf

    for f = 1 : numStates
        subplot(numStates, 1, f)

        % 2-sigma covariance bounds
        sigmaBound = 2 * sqrt(PX(f, :));

        % Shaded 2-sigma region
        fill([T, fliplr(T)], [sigmaBound, fliplr(-sigmaBound)], ...
             [1 0.8 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.5)
        hold on

        % 2-sigma boundary lines
        plot(T, sigmaBound, 'r--', 'LineWidth', 1.5)
        plot(T, -sigmaBound, 'r--', 'LineWidth', 1.5)

        % State error
        stateError = XEst(f, :) - XTrue(f, :);
        if f == 3
            % Wrap heading error to [-pi, pi]
            stateError = mod(stateError + pi, 2*pi) - pi;
        end
        plot(T, stateError, 'b-', 'LineWidth', 1.5)

        hold off
        grid on

        % Auto-scale axes based on data
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

    sgtitle('Q1b: Estimation Error with $2\sigma$ Covariance Bounds', ...
            'Interpreter', 'latex', 'FontSize', 14)

end
