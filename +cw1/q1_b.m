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


    fprintf('\n Q1b Results \n');

    % Compute RMSE for each state
    posError = XEst(1:2, :) - XTrue(1:2, :);
    headingError = mod(XEst(3,:) - XTrue(3,:) + pi, 2*pi) - pi;
    rmseX = sqrt(mean(posError(1,:).^2));
    rmseY = sqrt(mean(posError(2,:).^2));
    rmseH = sqrt(mean(headingError.^2));

    fprintf('RMSE x:       %.4f m\n', rmseX);
    fprintf('RMSE y:       %.4f m\n', rmseY);
    fprintf('RMSE heading: %.4f rad (%.2f deg)\n', rmseH, rad2deg(rmseH));
    fprintf('\n');

    % Final covariance bounds
    fprintf('Final 2-sigma x:   %.4f m\n', 2*sqrt(PX(1,end)));
    fprintf('Final 2-sigma y:   %.4f m\n', 2*sqrt(PX(2,end)));
    fprintf('Final 2-sigma psi: %.4f rad\n', 2*sqrt(PX(3,end)));
    fprintf('\n');

    % Estimator consistency: percentage of errors within 2-sigma
    allErrors = [posError(1,:); posError(2,:); headingError];
    allSigma = 2 * sqrt(PX);
    withinBounds = abs(allErrors) <= allSigma;
    for f = 1 : numStates
        pct = 100 * sum(withinBounds(f,:)) / numTimeSteps;
        fprintf('%s within 2-sigma: %.1f%% (expected ~95.4%%)\n', ...
                stateLabels{f}, pct);
    end

    fprintf('================================\n\n');
end
