% This class implements an event-based estimation system using g2o and
% the barebones for building up a minimal, ideal SLAM system. The system is
% event-based and responds to a sequence of events which are time stamped
% and served in order. To implement your SLAM system, you will need to
% implement various methods which mostly involve working with the graph.
% These methods are initially stubbed out and will generate exceptions if
% you try to call them.

classdef DriveBotSLAMSystem < minislam.slam.SLAMSystem
    
    properties(Access = public, Constant)
        % Platform state dimension
        NP = 3;
        
        % Landmark dimension
        NL = 2;
        
        % Initial cache size; might help a bit with performance
        INITIAL_CACHE_SIZE = 10000;
    end
    
    properties(Access = protected)
        
        % The most recently created vehicle vertex.
        currentVehicleVertex;
        
        % The set of all vertices associated with the vehicle state over
        % time.
        vehicleVertices;
        vehicleVertexId;
        
        % The set of all prediction edges. These are removed from the graph
        % afterwards if we don't use prediction
        processModelEdges;
        numProcessModelEdges;
        
        % The landmark vertices. Confusingly enough, "Map" here refers to
        % the data structure which is used to store the landmarks. (It
        % allows random access of landmarkID to landmark object.)
        landmarkIDStateVectorMap;
        
        % How often we recommend running the optimization
        recommendOptimizationPeriod;
        
        % Flag to show if we should prune the edges. This is needed for
        % question Q3a
        removePredictionEdgesFromGraph;
        keepFirstPredictionEdge;

        %Q3b
        %flag for pruning in Q3b
        pruneGraph;
        %maximum number of observed landmark edges we keep per vehicle vertex after
        %pruning
        pruningLandmarkMaxEdges;
        
    end
    
    methods(Access = public)
        
        % Create the localization system and start it up.
        function this = DriveBotSLAMSystem(configuration)
            
            % Call the base class constructor
            this = this@minislam.slam.SLAMSystem(configuration);
            
            % Preallocate for convenience
            this.vehicleVertices = cell(1, this.INITIAL_CACHE_SIZE);
            
            % No vehicle vertices initally set
            this.vehicleVertexId = 0;
            
            % The set of prediction edges, initially empty
            this.processModelEdges = cell(1, this.INITIAL_CACHE_SIZE);
            this.numProcessModelEdges = 0;
            
            % Allocate the landmark map
            this.landmarkIDStateVectorMap = containers.Map('KeyType', 'int64', 'ValueType', 'any');
            
            % By default, run very infrequently
            this.recommendOptimizationPeriod = inf;
            
            this.removePredictionEdgesFromGraph = false;
            this.keepFirstPredictionEdge = false;

            %Q3b
            %by default do not prune 
            this.pruneGraph = false;
            %keep all landmark edges by default
            this.pruningLandmarkMaxEdges = inf;
        end
        
        % Destroy the graph when we destroy the SLAM system.
        % Without this, MATLAB will crash whenever this object is destroyed.

        function delete(this)
            vertices = this.graph.vertices();

            for v = 1 : length(vertices)
                this.graph.removeVertex(vertices{v});
            end
        end
        
        % Recommend if an optimization is a good idea. Based on an event,
        % some activities (e.g., such as loop closing) can have a very big
        % impact on the estimates. The logic we have here just recommends
        % an optimization if a fixed number of steps have been completed.
        
        function recommendation = recommendOptimization(this)
            
            % This is how to do it after every 100 steps
            recommendation = rem(this.stepNumber, ...
                this.recommendOptimizationPeriod) == 0;
        end
        
        % Set the value of how often recommend optimization should return
        % true
        function setRecommendOptimizationPeriod(this, newRecommendOptimizationPeriod)
            this.recommendOptimizationPeriod = newRecommendOptimizationPeriod;
        end
        
        % Return the current mean and covariance estimate of the robot.
        % This is only valid after optimization has been called.
        function [x, P] = platformEstimate(this)
            [xS, PS] = this.graph.computeMarginals(this.currentVehicleVertex);
            x=full(xS);
            P=full(PS);
        end
        
        % Returns the entire history of the platform estimates. Suppose
        % there are n vehicle vertices. T is a 1 by N dimensional vector of
        % timesteps. X is a 3 by N dimensional vector of vehicle state (x,
        % y, theta). P is a 3 by N dimensional vector where the nth column
        % are the diagonals from the covariance matrix.
        function [T, X, P] = platformEstimateHistory(this)
            
            % Extract the graph
            [xS, PS] = this.graph.computeMarginals();
            
            % Create the output array
            X = zeros(this.NP, this.vehicleVertexId);
            P = zeros(this.NP, this.vehicleVertexId);
            T = zeros(1, this.vehicleVertexId);
            
            % Copy the outputs over
            for v = 1 : this.vehicleVertexId
                idx = this.vehicleVertices{v}.hessianIndex();
                
                T(v) = this.vehicleVertices{v}.time();
                
                % Copy the estimate into the array. If the vertices is
                % fixed (conditioned), its estimate is okay. The covariance
                % is not explicitly defined, but has a value of zero.
                % Therefore we fill this manually.
                if (isempty(idx) == true)
                    X(:, v) = this.vehicleVertices{v}.estimate();
                    P(:, v) = zeros(3, 1);
                else
                    X(:, v) = full(xS(idx));
                    P(:, v) = full(diag(PS(idx, idx)));
                end
            end
        end
        
        % Return the means and covariances of the landmark estimates. These
        % are only valid after optimization has been called.
        function [x, P, landmarkIds] = landmarkEstimates(this)
            
            landmarkVertices = values(this.landmarkIDStateVectorMap);
            
            numberOfLandmarks = length(landmarkVertices);
            
            landmarkIds = NaN(1, numberOfLandmarks);
            x = NaN(this.NL, numberOfLandmarks);
            P = NaN(this.NL, this.NL, numberOfLandmarks);
            
            [xS, PS] = this.graph.computeMarginals();
            
            for l = 1 : numberOfLandmarks
                landmarkIds(l) = landmarkVertices{l}.landmarkId();
                idx = landmarkVertices{l}.hessianIndex();
                x(:, l) = full(xS(idx));
                if (isempty(idx == true))
                    P(:, :, l) = zeros(3, 3);
                else
                    P(:, :, l) = full(PS(idx, idx));
                end
            end
        end
        
        % We overload the optimize method so that you can add additional
        % logic here
        function chi2 = optimize(this, maximumNumberOfOptimizationSteps)
            
            % Remove the prediction edges if requested.
            if (this.removePredictionEdgesFromGraph == true)
                this.deleteVehiclePredictionEdges();
            end

            %Q3b
            %Prune if requested
            if(this.pruneGraph==true)
                this.pruningFunction();
            end
            
            % Now call the actual optimizer. Let it handle the default if
            % no steps are specified.
            if (nargin > 1)
                chi2 = optimize@minislam.slam.SLAMSystem(this, ...
                    maximumNumberOfOptimizationSteps);
            else
                chi2 = optimize@minislam.slam.SLAMSystem(this);
            end
        end
        
        function setRemovePredictionEdges(this, removeEdges, keepFirst)
            this.removePredictionEdgesFromGraph = removeEdges;
            this.keepFirstPredictionEdge = keepFirst;
            
        end

        %Q3b
        %function to set whether we prune and the max number of landmarks
        %edges kept at each vehicle vertex
        function setPruningFlagAndMaxLandmarks(this, pruneGraph, pruningLandmarkMaxEdges)
            this.pruneGraph = pruneGraph;
            this.pruningLandmarkMaxEdges = pruningLandmarkMaxEdges;
        end
    end
    
    % These are the methods you will need to overload
    methods(Access = protected)
        
        % Handle the initial condition
        
        function handleInitialConditionEvent(this, event)
            
            % Create the first vertex, set its estimate to the initial
            % value and add it to the graph.
            this.currentVehicleVertex = drivebot.graph.VehicleStateVertex(this.currentTime);
            this.currentVehicleVertex.setEstimate(event.data);
            this.graph.addVertex(this.currentVehicleVertex);
            
            % Set the book keeping for this initial vertex.
            this.vehicleVertexId = 1;
            this.vehicleVertices{this.vehicleVertexId} = this.currentVehicleVertex;
            
            % If the covariance is 0, the vertex is known perfectly and so
            % we set it as fixed. If the covariance is non-zero, add a
            % unary initial prior condition edge instead. This adds a soft
            % constraint on where the state can be.
            if (det(event.covariance) < 1e-6)
                this.currentVehicleVertex.setFixed(true);
            else
                initialPriorEdge = drivebot.graph.InitialPriorEdge();
                initialPriorEdge.setMeasurement(event.data);
                initialPriorEdge.setInformation(inv(event.covariance));
                initialPriorEdge.setVertex(this.currentVehicleVertex);
                this.graph.addEdge(initialPriorEdge);
            end
        end
        
        function handleNoPrediction(~)
            % Nothing to do
        end
        
        function handleHeartbeatEvent(this, ~)
            % Nothing to do
        end
        
        function handlePredictToTime(this, time, dT)

            % Create the next vehicle vertex and add it to the graph
            
            this.currentVehicleVertex = drivebot.graph.VehicleStateVertex(time);
            
            % Q1b:
            %PROBLEM1: edge, vertex creation and estimate setting missing
            % Implement prediction code here
            edge = drivebot.graph.VehicleKinematicsEdge(dT); %create edge
            %set vertices for edge
            edge.setVertex(1, this.vehicleVertices{this.vehicleVertexId}); %previous edge
            edge.setVertex(2, this.currentVehicleVertex); %current edge
            
            % The vehicle control inputs. These are wheel speed and steer
            % angle. These are assumed to be held constant until the next
            % control input event comes along.
            edge.setMeasurement(this.u);
            edge.setInformation(inv(this.uCov));
            edge.initialize();%run proces model by calling initialize()
            
            this.currentVehicleVertex.setEstimate(edge.vertex(2).estimate()); %update vertex estimate by extracting from edge
            %add vertex and edge to graph
            this.graph.addEdge(edge);
            this.graph.addVertex(this.currentVehicleVertex);

            %update edges data
            this.numProcessModelEdges = this.numProcessModelEdges + 1;
            this.processModelEdges{this.numProcessModelEdges} = edge;

            %PROBLEM1 SOLVED BUT NEW PROBLEM2 found: in
            %VehicleKinematicsEdge; computeError and linearizeOplus
            %unimplemented
            %PROBLEM 2 solved by adapting implementation from
            %Workshop_Topic_02_Part_02_-_Factor_Graphs_and_Estimation 
            
         
%             warning('drivebotslam:handlepredicttotime:unimplemented', ...
%                 'Implement the rest of this method for Q1b.');
            
            % Bump the indices
            this.vehicleVertexId = this.vehicleVertexId + 1;
            this.vehicleVertices{this.vehicleVertexId} = this.currentVehicleVertex;
        end
        
        function handleGPSObservationEvent(this, event)

            % Q1d:
            % Create a GPS measurement edge and add it to the graph
            %created edge while considering Position offset of the GPS (in platform frame)
            GPSMeasurementEdge = drivebot.graph.GPSMeasurementEdge(this.configuration.gpsPositionOffset);
            %attach to vertex
            GPSMeasurementEdge.setVertex(1, this.currentVehicleVertex);
            %setting measurement and info
            GPSMeasurementEdge.setMeasurement(event.data);
            GPSMeasurementEdge.setInformation(inv(event.covariance));
            %adding edge to graph
            this.graph.addEdge(GPSMeasurementEdge);
%             warning('drivebotslam:handlegpsobservationevent:unimplemented', ...
%                 'Implement the rest of this method for Q1d.');
        end
        
        function handleCompassObservationEvent(this, event)
            
            % Create a compass measurement edge and add it to the graph
            compassMeasurementEdge = drivebot.graph.CompassMeasurementEdge(this.configuration.compassAngularOffset);
            compassMeasurementEdge.setVertex(1, this.currentVehicleVertex);
            compassMeasurementEdge.setMeasurement(event.data);
            compassMeasurementEdge.setInformation(inv(event.covariance));
            this.graph.addEdge(compassMeasurementEdge);
        end
        
        function handleLandmarkObservationEvent(this, event)
            
            % Iterate over all the landmark measurements
            for l = 1 : length(event.landmarkIds)
                
                % Get the landmark vertex associated with this measurement.
                % If necessary, a new landmark vertex is created and added
                % to the graph.
                [landmarkVertex, newVertexCreated] = this.createOrGetLandmark(event.landmarkIds(l));
                z = event.data(:, l);

                % Q2b:
                % Complete the implementation
                %create edge
                landmarkRangeBearingEdge = drivebot.graph.LandmarkRangeBearingEdge();
                %link to vertices
                landmarkRangeBearingEdge.setVertex(1, this.currentVehicleVertex);
                landmarkRangeBearingEdge.setVertex(2, landmarkVertex);
                %set measurement and info
                landmarkRangeBearingEdge.setMeasurement(z);
                landmarkRangeBearingEdge.setInformation(inv(event.covariance));
                %call initialize if we've just set up a new landmark so
                %that we estimate the coordinates
                if(newVertexCreated)
                    landmarkRangeBearingEdge.initialize();
                end
                


%                 warning('drivebotslamsystem:handlelandmarkobservationevent:unimplemented', ...
%                     'Implement the rest of this method for Q2b.');
                
                this.graph.addEdge(landmarkRangeBearingEdge);
            end
        end
        
        function deleteVehiclePredictionEdges(this)

            % Q3a:     
            allEdges = this.graph.edges(); %all edges
            flagFirstEdge = true; %flag to track whether we're on first edge

            %cycle through edges
            for i = 1:length(allEdges)
                
                %check for vehicle kinematics edges
                if(class(allEdges{i}) == "drivebot.graph.VehicleKinematicsEdge")
                    
                    %first edge found check whether we keep it
                    if(flagFirstEdge)
                        flagFirstEdge=false; %once we found first edge we cant find it again
                        if(this.keepFirstPredictionEdge)
                            continue
                        end
                    end
                    %remove vehicle kinematics edges
                    this.graph.removeEdge(allEdges{i});
                end
                
            end

%             warning('drivebotslam:deletevehiclepredictionedges:unimplemented', ...
%                 'Implement the rest of this method for Q3a.');
        end

        %Q3b
        %pruning function
        function pruningFunction(this)
            
            %get all vertices
            allVertices = this.graph.vertices();
            
            %cycle through them
            for i = 1 : length(allVertices)
                %if we get a vehicle vertex
                if(class(allVertices{i}) == "drivebot.graph.VehicleStateVertex")
                    
                    %if we have less edges than the max number, then do
                    %nothing
                    if(allVertices{i}.numberOfEdges() <= this.pruningLandmarkMaxEdges)
                        continue
                    end

                    %if we have more edges than we want, find the landmark
                    %edges
                    landmarkEdgesPruning = allVertices{i}.edges();

                    %cycle through to make sure we have all landmark edges
                    removeTheseIndices = [];
                    for j = 1 : length(landmarkEdgesPruning)
                        if(class(landmarkEdgesPruning{j}) ~= "drivebot.graph.LandmarkRangeBearingEdge")
                            removeTheseIndices = [removeTheseIndices, j];
                        end
                    end
                    %remove non landmark edges from cell array
                    landmarkEdgesPruning(removeTheseIndices) = [];
                    
                    %again check if we have more landmarks than needed
                    if(length(landmarkEdgesPruning)<=this.pruningLandmarkMaxEdges)
                        continue
                    end

                    %if we have more than needed then we cycle through the
                    %landmark edges and randomly remove edges until
                    %pruningLandmarkMaxEdges landmark edges left
                    length(landmarkEdgesPruning);
                    indices_to_not_remove_from_graph = randperm(length(landmarkEdgesPruning));
                    indices_to_not_remove_from_graph = indices_to_not_remove_from_graph(1:this.pruningLandmarkMaxEdges);
                    
                    %keep edges we want to remove from graph in the cell
                    %array
                    landmarkEdgesPruning(indices_to_not_remove_from_graph) = [];
                       
                    %now we remove the extra landmark edges
                    for k = 1 : length(landmarkEdgesPruning)
                        %associated_vehicle_vertex = landmarkEdgesPruning{k}.vertices()
                        %associated_vehicle_vertex = associated_vehicle_vertex{1}
                        %associated_vehicle_vertex.numberOfEdges()
                        this.graph.removeEdge(landmarkEdgesPruning{k});
                        %associated_vehicle_vertex.numberOfEdges()
                        
                    end
                    

                end
            end

        end
        
        
        % This method returns a landmark associated with landmarkId. If a
        % landmark exists already, it is returned. If it does not exist, a
        % vertex is created and is added to the graph.
        function [landmarkVertex, newVertexCreated] = createOrGetLandmark(this, landmarkId)

            
            % If the landmark exists already, return it
            if (isKey(this.landmarkIDStateVectorMap, landmarkId) == true)
                landmarkVertex = this.landmarkIDStateVectorMap(landmarkId);
                newVertexCreated = false;
                return
            end
            
            fprintf('Creating landmark %d\n', landmarkId);
            
            % Create the new landmark add it to the graph
            landmarkVertex = drivebot.graph.LandmarkStateVertex(landmarkId);
            this.landmarkIDStateVectorMap(landmarkId) = landmarkVertex;
            
            this.graph.addVertex(landmarkVertex);
            
            newVertexCreated = true;
        end
        
        function storeStepResults(this)
            % Nothing
        end
        
    end
end
