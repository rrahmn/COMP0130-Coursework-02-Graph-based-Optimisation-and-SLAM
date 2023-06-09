% This script runs Q2(c)

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
simulator = drivebot.DriveBotSimulator(configuration, 'q2_c');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);
drivebotSLAMSystem.setRecommendOptimizationPeriod(inf);

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
plot(results{1}.optimizationTimes, '*')
hold on

% Plot the error curves
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')

% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')
hold on

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
hold on

% Plot chi2 values
minislam.graphics.FigureManager.getFigure('chi2 values');
clf
plot(results{1}.chi2Time, results{1}.chi2History)
hold on


% This is how to extract the graph from the optimizer
graph = drivebotSLAMSystem.optimizer();
%%
% This is how to extract cell arrays of the vertices and edges from the
% graph
allVertices = graph.vertices();
allEdges = graph.edges();

% Work out the number of vehicle poses and landmarks. 
numVehicleVertices = 0;
numLandmarks = 0;

landmarkObservationsPerVehicleVertex = 0;
observationsPerLandmarkVertex = 0;

% Q2c:
% Finish implementing the code to capture information about the graph
% structure.
for i = 1:length(allVertices)
    %cycle through vertices and check whether we have vehicle poses or
    %landmarks initialised
    numVehicleVertices = numVehicleVertices + (class(allVertices{1,i}) == "drivebot.graph.VehicleStateVertex");
    numLandmarks = numLandmarks + (class(allVertices{1,i}) == "drivebot.graph.LandmarkStateVertex");
end

fprintf("the number of vehicle poses stored: " + num2str(numVehicleVertices) +"\n")
fprintf("the number of landmarks initalized: " + num2str(numLandmarks)+"\n")

landmarkEdges = 0;
for i = 1:length(allEdges)
    %cycle through edges and check whether we have a landmark edge
    landmarkEdges = landmarkEdges + (class(allEdges{1,i})=="drivebot.graph.LandmarkRangeBearingEdge");
end
%each landmark edge is an observation. we divide by the total number of
%time steps to get the average per timestep
fprintf("the average number of observations made by a robot at each timestep: " + num2str(landmarkEdges/length(results{1, 1}.vehicleTrueStateTime))+"\n")

%each landmark edge is an observation. we divide by the total number of
%landmarks to get the average observations per landmark
fprintf("the average number of observations received by each landmark: " + num2str(landmarkEdges/numLandmarks)+"\n")

% warning('q2_c:unimplemented', ...
%         'Implement the rest of the graph query code for Q2c.')
