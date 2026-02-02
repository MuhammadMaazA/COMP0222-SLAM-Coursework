classdef PlatformPredictionEdge < g2o.core.BaseBinaryEdge
    % PlatformPredictionEdge summary of PlatformPredictionEdge
    %
    % This edge implements the prediction step for the vehicle process model.
    % It connects two consecutive VehicleStateVertex instances and encodes
    % the motion constraint from odometry.
    %
    % Process Model (from Appendix A.2):
    %   x_{k+1} = x_k + M(psi_k) * u_{k+1}
    %
    % where:
    %   M_k = dT * [cos(psi_k), -sin(psi_k), 0;
    %               sin(psi_k),  cos(psi_k), 0;
    %                       0,           0, 1]
    %
    %   u = [s; 0; psi_dot]  (speed, lateral speed=0, angular velocity)
    %
    % The measurement stored is the odometry input u.
    % The time interval dT is stored separately.
    
    properties(Access = protected)
        % Time interval for the prediction step
        dT;
    end
    
    methods(Access = public)
        
        function obj = PlatformPredictionEdge()
            % PlatformPredictionEdge Constructor
            %
            % Syntax:
            %   obj = PlatformPredictionEdge()
            %
            % Description:
            %   Creates a binary edge for the platform prediction step.
            %   The edge connects two VehicleStateVertex instances:
            %   - Vertex 1: previous pose x_k
            %   - Vertex 2: current pose x_{k+1}
            %
            %   The measurement dimension is 3 (matching the odometry u).
            
            obj = obj@g2o.core.BaseBinaryEdge(3);
            
            % Initialize dT to zero
            obj.dT = 0;
        end
        
        function setDT(obj, dT)
            % SETDT Set the time interval for the prediction step.
            %
            % Syntax:
            %   obj.setDT(dT)
            %
            % Inputs:
            %   dT - (double)
            %       The time interval between the two poses.
            
            obj.dT = dT;
        end
        
        function dT = getDT(obj)
            % GETDT Get the time interval for the prediction step.
            %
            % Syntax:
            %   dT = obj.getDT()
            %
            % Outputs:
            %   dT - (double)
            %       The time interval between the two poses.
            
            dT = obj.dT;
        end
        
        function initialEstimate(obj)
            % INITIALESTIMATE Initialize the estimate of the second vertex.
            %
            % Syntax:
            %   obj.initialEstimate()
            %
            % Description:
            %   Uses the process model to predict the state of vertex 2
            %   (x_{k+1}) given vertex 1 (x_k) and the odometry measurement.
            %   This is used to initialize new vertices when they are added.
            %
            %   The process model is:
            %   x_{k+1} = x_k + M(psi_k) * u
            %
            %   where M is the rotation matrix scaled by dT.
            
            % Get the current state from vertex 1
            x_k = obj.edgeVertices{1}.estimate();
            
            % Extract the heading
            psi_k = x_k(3);
            
            % Get the odometry measurement (u = [s; 0; psi_dot])
            u = obj.z;
            
            % Construct the M matrix (Equation A.4 from appendix)
            M = obj.dT * [cos(psi_k), -sin(psi_k), 0;
                          sin(psi_k),  cos(psi_k), 0;
                                  0,           0, 1];
            
            % Predict the next state
            x_kp1 = x_k + M * u;
            
            % Wrap the heading angle
            x_kp1(3) = g2o.stuff.normalize_theta(x_kp1(3));
            
            % Set the estimate for vertex 2
            obj.edgeVertices{2}.setEstimate(x_kp1);
        end
        
        function computeError(obj)
            % COMPUTEERROR Compute the error for this edge.
            %
            % Syntax:
            %   obj.computeError()
            %
            % Description:
            %   Computes the error in the body frame. The process model is:
            %     x_{k+1} = x_k + M(psi_k) * (u + v)
            %
            %   Rearranging to isolate the noise term v:
            %     v = M^{-1} * (x_{k+1} - x_k) - u
            %
            %   The error is therefore:
            %     error = M^{-1}(psi_k) * (x_{k+1} - x_k) - u
            %
            %   This body-frame formulation ensures the error covariance
            %   matches Q directly, consistent with information = inv(Q).

            % Get states from both vertices
            x_k = obj.edgeVertices{1}.estimate();
            x_kp1 = obj.edgeVertices{2}.estimate();

            % Extract the heading from vertex 1
            psi_k = x_k(3);

            % Get the odometry measurement
            u = obj.z;

            % Construct the inverse M matrix
            % M = dT * R, so M^{-1} = (1/dT) * R'
            c = cos(psi_k);
            s = sin(psi_k);
            invM = (1 / obj.dT) * [c,  s, 0;
                                   -s,  c, 0;
                                    0,  0, 1];

            % Compute the body-frame error
            obj.errorZ = invM * (x_kp1 - x_k) - u;

            % Wrap the heading error to [-pi, pi]
            obj.errorZ(3) = g2o.stuff.normalize_theta(obj.errorZ(3));
        end
        
        function linearizeOplus(obj)
            % LINEARIZEOPLUS Compute the Jacobians of the error function.
            %
            % Syntax:
            %   obj.linearizeOplus()
            %
            % Description:
            %   Computes the Jacobians of the error function with respect
            %   to the two vertices. The error is in body frame:
            %
            %     error = M^{-1}(psi_k) * (x_{k+1} - x_k) - u
            %
            %   J{2} = d(error)/d(x_{k+1}) = M^{-1}
            %
            %   J{1} = d(error)/d(x_k) = -M^{-1} + [0, 0, dM^{-1}/dpsi * delta]
            %
            %   where delta = x_{k+1} - x_k and:
            %     dM^{-1}/dpsi = (1/dT) * [-sin(psi), cos(psi), 0;
            %                               -cos(psi), -sin(psi), 0;
            %                                       0,         0, 0]

            % Get states from both vertices
            x_k = obj.edgeVertices{1}.estimate();
            x_kp1 = obj.edgeVertices{2}.estimate();
            psi_k = x_k(3);

            c = cos(psi_k);
            s = sin(psi_k);

            % Inverse M matrix: M^{-1} = (1/dT) * R'
            invM = (1 / obj.dT) * [c,  s, 0;
                                   -s,  c, 0;
                                    0,  0, 1];

            % State difference
            delta = x_kp1 - x_k;
            dx = delta(1);
            dy = delta(2);

            % Derivative of (M^{-1} * delta) w.r.t. psi_k
            dInvMdelta_dpsi = (1 / obj.dT) * ...
                [-s * dx + c * dy;
                 -c * dx - s * dy;
                                0];

            % Jacobian w.r.t. vertex 1 (x_k)
            obj.J{1} = -invM;
            obj.J{1}(:, 3) = obj.J{1}(:, 3) + dInvMdelta_dpsi;

            % Jacobian w.r.t. vertex 2 (x_{k+1})
            obj.J{2} = invM;
        end
    end
end
