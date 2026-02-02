classdef LandmarkRangeBearingEdge < g2o.core.BaseBinaryEdge
    % LandmarkRangeBearingEdge summary of LandmarkRangeBearingEdge
    %
    % This edge implements the landmark range-bearing observation model.
    % It connects a VehicleStateVertex (platform pose) and a 
    % LandmarkStateVertex (landmark position).
    %
    % Landmark Observation Model (from Appendix A.4):
    %   z_L = [r; beta] + w_L
    %
    % where:
    %   r = sqrt((x_i - x)^2 + (y_i - y)^2)
    %   beta = atan2(y_i - y, x_i - x) - psi
    %
    % The measurement is [range; bearing] to the landmark.
    
    methods(Access = public)
        
        function obj = LandmarkRangeBearingEdge()
            % LandmarkRangeBearingEdge Constructor
            %
            % Syntax:
            %   obj = LandmarkRangeBearingEdge()
            %
            % Description:
            %   Creates a binary edge for landmark range-bearing observations.
            %   The edge connects:
            %   - Vertex 1: VehicleStateVertex (platform pose [x, y, psi])
            %   - Vertex 2: LandmarkStateVertex (landmark position [m_x, m_y])
            %
            %   The measurement dimension is 2 (range, bearing).
            
            obj = obj@g2o.core.BaseBinaryEdge(2);
        end
        
        function initialEstimate(obj)
            % INITIALESTIMATE Initialize the estimate of the landmark vertex.
            %
            % Syntax:
            %   obj.initialEstimate()
            %
            % Description:
            %   Uses the inverse observation model to compute the initial
            %   landmark position given the vehicle pose and the range-bearing
            %   measurement.
            %
            %   The inverse model is:
            %   phi = psi + beta
            %   m_x = x + r * cos(phi)
            %   m_y = y + r * sin(phi)
            %
            %   This is used to initialize new landmarks when they are first
            %   observed.
            
            % Get the platform state from vertex 1
            x_p = obj.edgeVertices{1}.estimate();
            x = x_p(1);
            y = x_p(2);
            psi = x_p(3);
            
            % Get the measurement
            r = obj.z(1);      % Range
            beta = obj.z(2);   % Bearing
            
            % Compute the angle in world frame
            phi = psi + beta;
            
            % Compute landmark position using inverse observation model
            m_x = x + r * cos(phi);
            m_y = y + r * sin(phi);
            
            % Set the estimate for vertex 2 (landmark)
            obj.edgeVertices{2}.setEstimate([m_x; m_y]);
        end
        
        function computeError(obj)
            % COMPUTEERROR Compute the error for this edge.
            %
            % Syntax:
            %   obj.computeError()
            %
            % Description:
            %   Computes the error between the measured range-bearing and
            %   the predicted range-bearing from the current state estimates.
            %
            %   error = z - h(x, m)
            %
            %   where h(x, m) = [r; beta]:
            %   r = sqrt((m_x - x)^2 + (m_y - y)^2)
            %   beta = atan2(m_y - y, m_x - x) - psi
            %
            %   Note: The bearing error is wrapped to [-pi, pi].
            
            % Get the platform state from vertex 1
            x_p = obj.edgeVertices{1}.estimate();
            x = x_p(1);
            y = x_p(2);
            psi = x_p(3);
            
            % Get the landmark state from vertex 2
            m = obj.edgeVertices{2}.estimate();
            m_x = m(1);
            m_y = m(2);
            
            % Compute the relative position
            dx = m_x - x;
            dy = m_y - y;
            
            % Compute predicted range
            r_pred = sqrt(dx^2 + dy^2);
            
            % Compute predicted bearing (angle to landmark in robot frame)
            beta_pred = atan2(dy, dx) - psi;
            
            % Compute the error
            obj.errorZ = obj.z - [r_pred; beta_pred];
            
            % Wrap the bearing error to [-pi, pi]
            obj.errorZ(2) = g2o.stuff.normalize_theta(obj.errorZ(2));
        end
        
        function linearizeOplus(obj)
            % LINEARIZEOPLUS Compute the Jacobians of the error function.
            %
            % Syntax:
            %   obj.linearizeOplus()
            %
            % Description:
            %   Computes the Jacobians of the error function with respect
            %   to the two vertices:
            %
            %   J{1} = d(error)/d(platform_state) = -d(h)/d([x, y, psi])
            %   J{2} = d(error)/d(landmark_state) = -d(h)/d([m_x, m_y])
            %
            %   From the observation model h = [r; beta]:
            %
            %   d(h)/d([x, y, psi]) = [-dx/r,  -dy/r,   0;
            %                          dy/r^2, -dx/r^2, -1]
            %
            %   d(h)/d([m_x, m_y]) = [dx/r,   dy/r;
            %                        -dy/r^2, dx/r^2]
            %
            %   where dx = m_x - x, dy = m_y - y, r = sqrt(dx^2 + dy^2)
            
            % Get the platform state from vertex 1
            x_p = obj.edgeVertices{1}.estimate();
            x = x_p(1);
            y = x_p(2);
            
            % Get the landmark state from vertex 2
            m = obj.edgeVertices{2}.estimate();
            m_x = m(1);
            m_y = m(2);
            
            % Compute relative position
            dx = m_x - x;
            dy = m_y - y;
            
            % Compute range and squared range
            r2 = dx^2 + dy^2;
            r = sqrt(r2);
            
            % Numerical stability: avoid division by zero
            if r < 1e-6
                r = 1e-6;
                r2 = r^2;
            end
            
            % Jacobian of h w.r.t. platform state [x, y, psi]
            % h = [r; beta]
            % d(r)/d(x) = -dx/r, d(r)/d(y) = -dy/r, d(r)/d(psi) = 0
            % d(beta)/d(x) = dy/r^2, d(beta)/d(y) = -dx/r^2, d(beta)/d(psi) = -1
            gradHx = [-dx/r,  -dy/r,   0;
                       dy/r2, -dx/r2, -1];
            
            % Jacobian of h w.r.t. landmark state [m_x, m_y]
            % d(r)/d(m_x) = dx/r, d(r)/d(m_y) = dy/r
            % d(beta)/d(m_x) = -dy/r^2, d(beta)/d(m_y) = dx/r^2
            gradHm = [dx/r,   dy/r;
                     -dy/r2,  dx/r2];
            
            % Error = z - h, so d(error)/d(.) = -d(h)/d(.)
            obj.J{1} = -gradHx;
            obj.J{2} = -gradHm;
        end
    end
end
