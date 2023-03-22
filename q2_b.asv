% This script runs Q2(b)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Enable the laser to support pure SLAM
configuration.enableLaser = true;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = true;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q2_b');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);
drivebotSLAMSystem.setRecommendOptimizationPeriod(5);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.



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
saveas(gcf, 'latex16', 'png');
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
saveas(gcf, 'latex17', 'png');


% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.vehicleStateTime, results{1}.optimizationTimes, '*')
hold on
xlabel("Time axis")
ylabel("Time taken for optimisation seconds")
title("optimisation times vs time")
saveas(gcf, 'latex18', 'png');


% % Plot errors
% minislam.graphics.FigureManager.getFigure('Errors');
% clf
% plot(results{1}.vehicleStateTime, results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
% hold on
% %labels and legend
% xlabel("Time")
% ylabel("Error")
% legend('x', 'y', 'phi', 'Location','best')
% %saving
% saveas(gcf, 'latex19', 'png');





