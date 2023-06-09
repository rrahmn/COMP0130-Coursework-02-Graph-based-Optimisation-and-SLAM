% This script runs Q2(d)
rng(123); %to make results repeatable by setting random seed. remove this 
% line to get different results each time .


% Create the configuration object.
configurationBeforeLoopClosure = drivebot.SimulatorConfiguration();
configurationAfterLoopClosure = drivebot.SimulatorConfiguration();

% Enable the laser to support pure SLAM
configurationBeforeLoopClosure.enableLaser = true;
configurationAfterLoopClosure.enableLaser = true;

% For this part of the coursework, this should be set to false.
configurationBeforeLoopClosure.perturbWithNoise = false;
configurationAfterLoopClosure.perturbWithNoise = false;

% Set this value to truncate the run at a specified timestep rather than
% run through the whole simulation to its end.
configurationBeforeLoopClosure.maximumStepNumber = 1206; %just before loop closure, found by trial and error
configurationAfterLoopClosure.maximumStepNumber = 1209; %just after loop closure, found by trial and error

% Set up the simulator
simulatorBeforeLoopClosure = drivebot.DriveBotSimulator(configurationBeforeLoopClosure, 'q2_d');
simulatorAfterLoopClosure = drivebot.DriveBotSimulator(configurationAfterLoopClosure, 'q2_d');

% Create the localization system
drivebotSLAMSystemBeforeLoopClosure = drivebot.DriveBotSLAMSystem(configurationBeforeLoopClosure);
drivebotSLAMSystemBeforeLoopClosure.setRecommendOptimizationPeriod(inf);
drivebotSLAMSystemAfterLoopClosure = drivebot.DriveBotSLAMSystem(configurationAfterLoopClosure);
drivebotSLAMSystemAfterLoopClosure.setRecommendOptimizationPeriod(inf);

% Q2d:
% Explore the  timestep where the loop closure occurs, and get
% results just before and after the loop closure event
% warning('q2_d:unimplemented', ...
%         'Analyse loop closure behaviour for Q2d.')

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystemBeforeLoopClosure.setValidateGraph(false);
drivebotSLAMSystemAfterLoopClosure.setValidateGraph(false);

% Run the main loop and correct results

%saves plot before loop closure
resultsBeforeLoopClosure = minislam.mainLoop(simulatorBeforeLoopClosure, drivebotSLAMSystemBeforeLoopClosure);
title("map, vehicle and landmark states just before loop closure");
saveas(gcf, 'latex19_before_loop_closure', 'png');

%saves plot after loop closure
resultsAfterLoopClosure = minislam.mainLoop(simulatorAfterLoopClosure, drivebotSLAMSystemAfterLoopClosure);
title("map, vehicle and landmark states just after loop closure");
saveas(gcf, 'latex19_after_loop_closure', 'png');


%landmark covariances
[~, covariance_history_BeforeLoopClosure, ~] = drivebotSLAMSystemBeforeLoopClosure.landmarkEstimates();
[~, covariance_history_AfterLoopClosure, ~] = drivebotSLAMSystemAfterLoopClosure.landmarkEstimates();

for i = 1:length(covariance_history_AfterLoopClosure)
    fprintf("#########################\n")
    fprintf("landmark " + num2str(i) + "\n")

    fprintf("Determinant of landmark covariance before loop closure: ")
    det(covariance_history_BeforeLoopClosure(:,:,i))

    fprintf("Determinant of landmark covariance after loop closure: ")
    det(covariance_history_AfterLoopClosure(:,:,i))

    fprintf("Difference in determinant of landmark covariance before and after loop closure: ")
    -det(covariance_history_BeforeLoopClosure(:,:,i)) + det(covariance_history_AfterLoopClosure(:,:,i))

    fprintf("Percentage change in determinant of landmark covariance before and after loop closure: ")
    100*(-det(covariance_history_BeforeLoopClosure(:,:,i)) + det(covariance_history_AfterLoopClosure(:,:,i)))/det(covariance_history_BeforeLoopClosure(:,:,i))
    fprintf("#########################\n")
end