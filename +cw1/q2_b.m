import ebe.core.*;
import ebe.graphics.*;
import cw1.*;

% Find, load and parse the configuration file
config = ebe.utils.readJSONFile('config/q2_b.json');

% Create the mainloop object
mainLoop = ebe.MainLoop(config);

% Create the simulator and register it
simulator = drivebot.Simulator(config);
mainLoop.setEventGenerator(simulator);

% Create the G2O SLAM system and register it
g2oSLAMSystem = drivebot.G2OSLAMSystem(config);
mainLoop.addEstimator(g2oSLAMSystem);

% Create the EKF SLAM system and register it (for comparison)
ekfSLAMSystem = drivebot.EKFSLAMSystem(config);
mainLoop.addEstimator(ekfSLAMSystem);

% Create the results accumulator
resultsAccumulator = ebe.slam.XPPlatformAccumulator();
mainLoop.addResultsAccumulator(resultsAccumulator);
mainLoop.setAccumulateResultsUpdatePeriod(25);

% Set up the live simulation figure
fig = FigureManager.getFigure('Q2b: Live Simulation');
clf
hold on
axis equal

% Set up the views
simulatorViewer = ebe.graphics.ViewManager(config);
simulatorView = drivebot.SimulatorView(config, simulator);
simulatorViewer.addView(simulatorView);
simulatorViewer.addView(drivebot.SLAMSystemView(config, ekfSLAMSystem));
simulatorViewer.addView(drivebot.SLAMSystemView(config, g2oSLAMSystem));

mainLoop.addViewer(simulatorViewer);
mainLoop.setGraphicsUpdatePeriod(25);

% Run the main loop
mainLoop.run();


T = resultsAccumulator.timeStore;
XTrue = resultsAccumulator.xTrueStore;
numStates = size(XTrue, 1);

stateLabels = {'$x$', '$y$', '$\psi$'};
yLabels = {'$x$ error (m)', '$y$ error (m)', '$\psi$ error (rad)'};
estimatorNames = {'G2O', 'EKF'};
colours = {'b', 'm'};

% Extract landmark positions from scenario config
lmPositions = config.scenario.landmarks.slam.landmarks;
if iscell(lmPositions)
    lmMat = cell2mat(lmPositions');
else
    lmMat = lmPositions;
end


ebe.graphics.FigureManager.getFigure('Q2b: Trajectory Comparison');
clf

% Plot landmarks
plot(lmMat(:,1), lmMat(:,2), 'k^', 'MarkerSize', 8, ...
     'MarkerFaceColor', [0.7 0.7 0.7], 'DisplayName', 'Landmarks')
hold on

% Plot ground truth
plot(XTrue(1,:), XTrue(2,:), 'g-', 'LineWidth', 2, 'DisplayName', 'Ground Truth')

% Plot each estimator
for e = 1 : numel(resultsAccumulator.xEstStore)
    XEst = resultsAccumulator.xEstStore{e};
    plot(XEst(1,:), XEst(2,:), [colours{e} '--'], 'LineWidth', 1.5, ...
         'DisplayName', estimatorNames{e})
end

% Start/end markers
plot(XTrue(1,1), XTrue(2,1), 'ko', 'MarkerSize', 10, ...
     'MarkerFaceColor', 'k', 'DisplayName', 'Start')
plot(XTrue(1,end), XTrue(2,end), 'rs', 'MarkerSize', 10, ...
     'MarkerFaceColor', 'r', 'DisplayName', 'End')
hold off
grid on
axis equal
xlabel('$x$ (m)', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$y$ (m)', 'Interpreter', 'latex', 'FontSize', 12)
title('Q2b: Trajectory Comparison with Landmarks', 'FontSize', 14)
legend('Location', 'best', 'FontSize', 9)
set(gca, 'FontSize', 11)


for e = 1 : numel(resultsAccumulator.xEstStore)
    PX = resultsAccumulator.PEstStore{e};
    XEst = resultsAccumulator.xEstStore{e};
    numTimeSteps = size(XEst, 2);

    ebe.graphics.FigureManager.getFigure( ...
        sprintf('Q2b: %s State Estimation Errors', estimatorNames{e}));
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
        if bound > 0
            axis([T(1) T(end) -bound bound])
        end

        xlabel('Time (s)', 'FontSize', 11)
        ylabel(yLabels{f}, 'Interpreter', 'latex', 'FontSize', 12)
        title(stateLabels{f}, 'Interpreter', 'latex', 'FontSize', 13)
        if f == 1
            legend('$2\sigma$ region', '$+2\sigma$', '$-2\sigma$', 'Error', ...
                   'Interpreter', 'latex', 'Location', 'best', 'FontSize', 9)
        end
        set(gca, 'FontSize', 10)
    end
    sgtitle(sprintf('Q2b: %s Estimation Error with $2\\sigma$ Bounds', ...
            estimatorNames{e}), 'Interpreter', 'latex', 'FontSize', 14)

    % Console summary
    fprintf('\n Q2b %s Results \n', estimatorNames{e});
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
    fprintf('==============================\n');
end


if numel(resultsAccumulator.xEstStore) >= 2
    XG2O = resultsAccumulator.xEstStore{1};
    XEKF = resultsAccumulator.xEstStore{2};
    nCommon = min(size(XG2O, 2), size(XEKF, 2));

    ebe.graphics.FigureManager.getFigure('Q2b: G2O vs EKF Comparison');
    clf

    discrepLabels = {'$\Delta x$ (m)', '$\Delta y$ (m)', '$\Delta\psi$ (rad)'};
    for f = 1 : numStates
        subplot(numStates, 1, f)
        discrepancy = XG2O(f, 1:nCommon) - XEKF(f, 1:nCommon);
        if f == 3
            discrepancy = mod(discrepancy + pi, 2*pi) - pi;
        end
        plot(T(1:nCommon), discrepancy, 'k-', 'LineWidth', 1.5)
        grid on
        xlabel('Time (s)', 'FontSize', 11)
        ylabel(discrepLabels{f}, 'Interpreter', 'latex', 'FontSize', 12)
        title(stateLabels{f}, 'Interpreter', 'latex', 'FontSize', 13)
        set(gca, 'FontSize', 10)
    end
    sgtitle('Q2b: G2O $-$ EKF State Discrepancy', ...
            'Interpreter', 'latex', 'FontSize', 14)

    fprintf('\n G2O vs EKF Discrepancy \n');
    for f = 1 : numStates
        d = XG2O(f, 1:nCommon) - XEKF(f, 1:nCommon);
        if f == 3
            d = mod(d + pi, 2*pi) - pi;
        end
        fprintf('Max |%s discrepancy|: %.6f\n', stateLabels{f}, max(abs(d)));
    end
    fprintf('==================================\n\n');
end


g2oPerfData = g2oSLAMSystem.getPerformanceData();
chi2Values = g2oPerfData.get('g2o.op.chi2');
optimDurations = g2oPerfData.get('g2o.op.op_dt');

numChi2 = numel(chi2Values);
numDur = numel(optimDurations);

ebe.graphics.FigureManager.getFigure('Q2b: Chi2 and Optimisation Time');
clf

subplot(2, 1, 1)
plot(1:numChi2, chi2Values, 'b-', 'LineWidth', 1.5)
grid on
xlabel('Optimisation step', 'FontSize', 11)
ylabel('$\chi^2$', 'Interpreter', 'latex', 'FontSize', 12)
title('Cost function ($\chi^2$) at each optimisation step', ...
      'Interpreter', 'latex', 'FontSize', 13)
set(gca, 'FontSize', 10)

subplot(2, 1, 2)
plot(1:numDur, optimDurations, 'r-', 'LineWidth', 1.5)
grid on
xlabel('Optimisation step', 'FontSize', 11)
ylabel('Duration (s)', 'FontSize', 12)
title('Optimisation duration at each step', 'FontSize', 13)
set(gca, 'FontSize', 10)

sgtitle('Q2b: $\chi^2$ and Optimisation Time', ...
        'Interpreter', 'latex', 'FontSize', 14)

fprintf(' Q2b: Chi2 Summary \n');
fprintf('Total optimisation steps:  %d\n', numChi2);
fprintf('Final chi2:                %.2f\n', chi2Values(end));
fprintf('Mean optimisation time:    %.4f s\n', mean(optimDurations));
fprintf('Max optimisation time:     %.4f s\n', max(optimDurations));
fprintf('=============================\n\n');
