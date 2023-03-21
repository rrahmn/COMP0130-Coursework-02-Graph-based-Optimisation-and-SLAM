classdef GPSMeasurementEdge < g2o.core.BaseUnaryEdge
   
    properties(Access = protected)
        
        xyOffset;
        
    end
    
    methods(Access = public)
    
        function this = GPSMeasurementEdge(xyOffset)
            this = this@g2o.core.BaseUnaryEdge(2);
            this.xyOffset = xyOffset;
        end
        
        function computeError(this)

	        % Q1d:
            % Implement the code
            %get the state values
            x = this.edgeVertices{1}.estimate();

            %set up inverse of M rotation matrix using heading
            c = cos(x(3));
            s = sin(x(3));
            %to invert M rotation
            Mi = [c s 0;
                -s c 0;
                0 0 1];
            %find error by rearranging and rememebering GPS is 2d
            this.errorZ = x(1:2) - this.z + Mi(1:2, 1:2)*this.xyOffset;
%             warning('gpsmeasurementedge:computeerror:unimplemented', ...
%                     'Implement the rest of this method for Q1d.');
        end
        
        function linearizeOplus(this)

	    % Q1d:
        % Implement the code
        this.J{1} = ...
                [1 0 0;
                0 1 0];
%         warning('gpsmeasurementedge:lineareizeoplus:unimplemented', ...
%                 'Implement the rest of this method for Q1d.');
        end
    end
end
