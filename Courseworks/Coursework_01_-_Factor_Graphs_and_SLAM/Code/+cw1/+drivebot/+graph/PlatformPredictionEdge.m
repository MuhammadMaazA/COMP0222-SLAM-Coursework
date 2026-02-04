classdef PlatformPredictionEdge < g2o.core.BaseBinaryEdge
    % PlatformPredictionEdge summary of PlatformPredictionEdge
    %
    % This class stores the factor representing the process model which
    % transforms the state from timestep k to k+1
    %
    % The process model is as follows.
    %
    % Define the rotation vector
    %
    %   M = dT * [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
    %
    % The new state is predicted from 
    %
    %   x_(k+1) = x_(k) + M * [vx;vy;theta]
    %
    % Note in this case the measurement is actually the mean of the process
    % noise. It has a value of 0. The error vector is given by
    %
    % e(x,z) = inv(M) * (x_(k+1) - x_(k))
    %
    % Note this requires estimates from two vertices - x_(k) and x_(k+1).
    % Therefore, this inherits from a binary edge. We use the convention
    % that vertex slot 1 contains x_(k) and slot 2 contains x_(k+1).
    
    properties(Access = protected)
        % The length of the time step
        dT;
    end
    
    methods(Access = public)
        function obj = PlatformPredictionEdge(dT)
            % PlatformPredictionEdge for PlatformPredictionEdge
            %
            % Syntax:
            %   obj = PlatformPredictionEdge(dT);
            %
            % Description:
            %   Creates an instance of the PlatformPredictionEdge object.
            %   This predicts the state from one timestep to the next. The
            %   length of the prediction interval is dT.
            %
            % Outputs:
            %   obj - (handle)
            %       An instance of a PlatformPredictionEdge

            assert(dT >= 0);
            obj = obj@g2o.core.BaseBinaryEdge(3);            
            obj.dT = dT;
        end
       
        function initialEstimate(obj)
            % INITIALESTIMATE Compute the initial estimate of a platform.
            %
            % Syntax:
            %   obj.initialEstimate();
            %
            % Description:
            %   Compute the initial estimate of the platform x_(k+1) given
            %   an estimate of the platform at time x_(k) and the control
            %   input u_(k+1)
            %
            %   Uses: x_(k+1) = x_(k) + M * u
            %   where M = dT * [cos(theta), -sin(theta), 0;
            %                   sin(theta),  cos(theta), 0;
            %                   0,           0,          1]

            % Get the state from vertex 1 (x_k)
            x_k = obj.edgeVertices{1}.x;
            theta = x_k(3);
            
            % Get the odometry measurement (u = [vx; vy; omega])
            u = obj.z;
            
            % Construct the M matrix
            M = obj.dT * [cos(theta), -sin(theta), 0;
                          sin(theta),  cos(theta), 0;
                          0,           0,          1];
            
            % Predict the next state: x_(k+1) = x_k + M * u
            x_kp1 = x_k + M * u;
            
            % Wrap the heading angle to [-pi, pi]
            x_kp1(3) = g2o.stuff.normalize_theta(x_kp1(3));
            
            % Set the estimate for vertex 2
            obj.edgeVertices{2}.setEstimate(x_kp1);
        end
        
        function computeError(obj)
            % COMPUTEERROR Compute the error for the edge.
            %
            % Syntax:
            %   obj.computeError();
            %
            % Description:
            %   Compute the value of the error using:
            %   e(x,z) = inv(M) * (x_(k+1) - x_(k)) - z
            %
            %   where inv(M) = (1/dT) * [cos(theta),  sin(theta), 0;
            %                           -sin(theta),  cos(theta), 0;
            %                            0,           0,          1]

            % Get states from both vertices
            x_k = obj.edgeVertices{1}.x;
            x_kp1 = obj.edgeVertices{2}.x;
            theta = x_k(3);
            
            % Compute the state difference
            dx = x_kp1 - x_k;
            
            % Construct inv(M) = (1/dT) * rotation matrix transpose
            c = cos(theta);
            s = sin(theta);
            invM = (1 / obj.dT) * [c,  s, 0;
                                   -s, c, 0;
                                   0,  0, 1];
            
            % Compute error: e = inv(M) * (x_(k+1) - x_k) - z
            obj.errorZ = invM * dx - obj.z;
            
            % Wrap the heading error to [-pi, pi]
            obj.errorZ(3) = g2o.stuff.normalize_theta(obj.errorZ(3));
        end
        
        % Compute the Jacobians
        function linearizeOplus(obj)
            % LINEARIZEOPLUS Compute the Jacobians for the edge.
            %
            % Syntax:
            %   obj.linearizeOplus();
            %
            % Description:
            %   Compute the Jacobians for the edge. Since we have two
            %   vertices which contribute to the edge, the Jacobians with
            %   respect to both of them must be computed.
            %
            %   e = inv(M) * (x_(k+1) - x_k) - z
            %
            %   J{1} = de/dx_k
            %   J{2} = de/dx_(k+1) = inv(M)

            % Get states from both vertices
            x_k = obj.edgeVertices{1}.x;
            x_kp1 = obj.edgeVertices{2}.x;
            theta = x_k(3);
            
            % Compute state difference
            dx = x_kp1 - x_k;
            
            % Trig values
            c = cos(theta);
            s = sin(theta);
            
            % inv(M) = (1/dT) * [c, s, 0; -s, c, 0; 0, 0, 1]
            invDT = 1 / obj.dT;
            
            % J{2} = de/dx_(k+1) = inv(M)
            obj.J{2} = invDT * [c,  s, 0;
                                -s, c, 0;
                                0,  0, 1];
            
            % J{1} = de/dx_k
            % First two columns: -inv(M)[:, 1:2]
            % Third column: d(inv(M))/dθ * dx - inv(M)[:, 3]
            %
            % d(inv(M))/dθ = (1/dT) * [-s, c, 0; -c, -s, 0; 0, 0, 0]
            % d(inv(M))/dθ * dx = (1/dT) * [-s*dx(1) + c*dx(2); -c*dx(1) - s*dx(2); 0]
            
            obj.J{1} = invDT * [-c, -s, -s*dx(1) + c*dx(2);
                                 s, -c, -c*dx(1) - s*dx(2);
                                 0,  0, -1];
        end
    end    
end