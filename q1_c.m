% This script runs Q1(c)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Q1c: Set the configuration to enable the compass

% Set the compass angular offset. DO NOT change this value.
configuration.compassAngularOffset=0.75*pi;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = true;

%q1c
%Modify the script q1 c.m so that the simulator will generate 
% compass measurements
configuration.enableCompass = true;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q1_c');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);

% Force the optimizer to run with this frequency. This lets you see what's
% happening in greater detail, but slows everything down.
drivebotSLAMSystem.setRecommendOptimizationPeriod(20);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(true);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.vehicleStateTime, results{1}.optimizationTimes, '*')
hold on
xlabel("Time step axis")
ylabel("Time taken for optimisation seconds")
saveas(gcf, 'latex4_after', 'png');

% Plot the error curves
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateTime, results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')


% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleStateTime, results{1}.vehicleCovarianceHistory')
hold on
%labels and legend
xlabel("Time")
ylabel("Covariance")
legend('x', 'y', 'phi', 'Location','best')
%saving
saveas(gcf, 'latex5_after', 'png');

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateTime, results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
hold on
%labels and legend
xlabel("Time")
ylabel("Error")
legend('x', 'y', 'phi', 'Location','best')
%saving
saveas(gcf, 'latex6_after', 'png');

% Plot chi2 values
minislam.graphics.FigureManager.getFigure('chi2');
clf
plot(results{1}.chi2Time, log(results{1}.chi2History))
hold on
%labels and legend
xlabel("Time")
ylabel("log of chi2")
%saving
saveas(gcf, 'latex7_after', 'png');

