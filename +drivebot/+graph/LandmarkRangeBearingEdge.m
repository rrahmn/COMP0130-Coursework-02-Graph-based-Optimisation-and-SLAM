% This edge encodes a 3D range bearing measurement.
%
% The measurement is in spherical polar coordinates

% Jacobian from https://github.com/petercorke/robotics-toolbox-matlab/blob/master/RangeBearingSensor.m

classdef LandmarkRangeBearingEdge < g2o.core.BaseBinaryEdge
    
    methods(Access = public)
    
        function this = LandmarkRangeBearingEdge()
            this = this@g2o.core.BaseBinaryEdge(2);
        end
        
        function initialize(this)
            % Q2b:
            % Complete implementation
            %robot coordinates and rotaion
            x = this.edgeVertices{1}.estimate();
            %landmark coordinates
            u = x(1) + this.z(1)*cos(this.z(2) + x(3));
            v = x(2) + this.z(1)*sin(this.z(2) + x(3));
            %setting estimate of landmark position
            this.edgeVertices{2}.setEstimate([u;v])
%             warning('landmarkrangebearingedge:initialize:unimplemented', ...
%                 'Implement the rest of this method for Q1b.');
        end
        
        function computeError(this)

            % Q2b:
            % Complete implementation
            %Taken from
            %COMP0130_22-23_Topic_02\Workshop_Topic_02_Part_02_-_Factor_Graphs_and_Estimation\Solution\+odometry_model_answer\LandmarkRangeBearingEdge.m
            x = this.edgeVertices{1}.estimate();
            dx = this.edgeVertices{2}.estimate() - x(1:2);
            
            this.errorZ(1) = norm(dx) - this.z(1);
            this.errorZ(2) = g2o.stuff.normalize_theta(atan2(dx(2), dx(1)) - x(3) - this.z(2));
%             warning('landmarkrangebearingedge:computeerror:unimplemented', ...
%                 'Implement the rest of this method for Q1b.');

        end
        
        function linearizeOplus(this)
            % Q2b:
            % Complete implementation
            %Taken from
            %COMP0130_22-23_Topic_02\Workshop_Topic_02_Part_02_-_Factor_Graphs_and_Estimation\Solution\+odometry_model_answer\LandmarkRangeBearingEdge.m
            x = this.edgeVertices{1}.estimate();
            dx = this.edgeVertices{2}.estimate() - x(1:2);
            r = norm(dx);
            
            this.J{1} = ...
                [-dx(1)/r -dx(2)/r 0;
                dx(2)/r^2 -dx(1)/r^2 -1];
            this.J{2} = - this.J{1}(1:2, 1:2);
%             warning('landmarkrangebearingedge:linearizeoplus:unimplemented', ...
%                 'Implement the rest of this method for Q1b.');
        end        
    end
end