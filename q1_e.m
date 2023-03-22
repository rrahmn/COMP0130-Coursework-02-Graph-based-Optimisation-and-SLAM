% This script runs Q1(e)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Since we are doing prediction and GPS, disable the SLAM sensor
configuration.enableGPS = true;

% Set to true for part ii
configuration.enableCompass = true;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q1_e');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);

% Q1(e)i:
% Use the method "setRecommendOptimizationPeriod" in DriveBotSLAMSystem
% to control the rate at which the optimizer runs
drivebotSLAMSystem.setRecommendOptimizationPeriod(configuration.DT);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);




% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.vehicleStateTime, results{1}.optimizationTimes, '*')
hold on
xlabel("Time in seconds")
ylabel("Time taken for optimisation seconds")
title("Optimisation times vs time")
saveas(gcf, 'latex12_aftercompass', 'png');

% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleStateTime, results{1}.vehicleCovarianceHistory')
hold on
%labels and legend
xlabel("Time in seconds")
ylabel("Covariance")
legend('x', 'y', 'phi', 'Location','best')
title("Covariances vs time")
%saving
saveas(gcf, 'latex13_aftercompass', 'png');

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateTime, results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
hold on
%labels and legend
xlabel("Time in seconds")
ylabel("Error")
legend('x', 'y', 'phi', 'Location','best')
title("Errors vs time")
%saving
saveas(gcf, 'latex14_aftercompass', 'png');

% Plot chi2 values
minislam.graphics.FigureManager.getFigure('chi2 values');
clf
plot(results{1}.chi2Time, results{1}.chi2History)
hold on
%labels and legend
xlabel("Time in seconds")
ylabel("log of chi2")
title("log of chi2 vs time")
%saving
saveas(gcf, 'latex15_aftercompass', 'png');


