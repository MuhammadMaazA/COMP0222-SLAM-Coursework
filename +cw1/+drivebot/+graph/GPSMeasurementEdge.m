classdef GPSMeasurementEdge < g2o.core.BaseUnaryEdge
    % GPSMeasurementEdge summary of GPSMeasurementEdge
    %
    % This edge implements the GPS observation model as a unary edge.
    % It connects to a single VehicleStateVertex and constrains the
    % (x, y) position based on GPS measurements.
    %
    % GPS Observation Model (from Appendix A.3):
    %   z_G = [x; y] + w_G
    %
    % where w_G is zero-mean Gaussian noise with covariance R_G.
    %
    % The measurement is the GPS reading [x_gps; y_gps].
    
    methods(Access = public)
        
        function obj = GPSMeasurementEdge()
            % GPSMeasurementEdge Constructor
            %
            % Syntax:
            %   obj = GPSMeasurementEdge()
            %
            % Description:
            %   Creates a unary edge for GPS measurements.
            %   The edge connects to a single VehicleStateVertex.
            %   The measurement dimension is 2 (x, y position).
            
            obj = obj@g2o.core.BaseUnaryEdge(2);
        end
        
        function computeError(obj)
            % COMPUTEERROR Compute the error for this edge.
            %
            % Syntax:
            %   obj.computeError()
            %
            % Description:
            %   Computes the error between the measured GPS position
            %   and the predicted position from the vertex state.
            %
            %   error = z - h(x) = [z_x; z_y] - [x; y]
            
            % Get the state from the vertex
            x = obj.edgeVertices{1}.estimate();
            
            % Predicted observation is just the position
            h_x = x(1:2);
            
            % Error is measurement - prediction
            obj.errorZ = obj.z - h_x;
        end
        
        function linearizeOplus(obj)
            % LINEARIZEOPLUS Compute the Jacobian of the error function.
            %
            % Syntax:
            %   obj.linearizeOplus()
            %
            % Description:
            %   Computes the Jacobian of the error function with respect
            %   to the vertex state.
            %
            %   J = d(error)/d(x) = d(z - h(x))/d(x) = -d(h(x))/d(x)
            %
            %   Since h(x) = [x; y], we have:
            %   d(h)/d(x) = [1, 0, 0; 0, 1, 0]
            %
            %   So J = -[1, 0, 0; 0, 1, 0]
            
            % Jacobian: d(error)/d(x) = -d(h)/d(x)
            % h = [x; y], so d(h)/d([x,y,psi]) = [1,0,0; 0,1,0]
            obj.J{1} = -[1, 0, 0;
                         0, 1, 0];
        end
    end
end
