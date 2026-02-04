classdef LandmarkRangeBearingEdge < g2o.core.BaseBinaryEdge
    % LandmarkRangeBearingEdge summary of LandmarkRangeBearingEdge
    %
    % This class stores an edge which represents the factor for observing
    % the range and bearing of a landmark from the vehicle. Note that the
    % sensor is fixed to the platform.
    %
    % The measurement model is
    %
    %    z_(k+1)=h[x_(k+1)]+w_(k+1)
    %
    % The measurements are r_(k+1) and beta_(k+1) and are given as follows.
    % The sensor is at (lx, ly).
    %
    %    dx = lx - x_(k+1); dy = ly - y_(k+1)
    %
    %    r(k+1) = sqrt(dx^2+dy^2)
    %    beta(k+1) = atan2(dy, dx) - theta_(k+1)
    %
    % The error term
    %    e(x,z) = z(k+1) - h[x(k+1)]
    %
    % However, remember that angle wrapping is required, so you will need
    % to handle this appropriately in compute error.
    %
    % Note this requires estimates from two vertices - x_(k+1) and l_(k+1).
    % Therefore, this inherits from a binary edge. We use the convention
    % that vertex slot 1 contains x_(k+1) and slot 2 contains l_(k+1).
    
    methods(Access = public)
    
        function obj = LandmarkRangeBearingEdge()
            % LandmarkRangeBearingEdge for LandmarkRangeBearingEdge
            %
            % Syntax:
            %   obj = LandmarkRangeBearingEdge();
            %
            % Description:
            %   Creates an instance of the LandmarkRangeBearingEdge object.
            %   Note we feed in to the constructor the landmark position.
            %   This is to show there is another way to implement this
            %   functionality from the range bearing edge from activity 3.
            %
            % Outputs:
            %   obj - (handle)
            %       An instance of a LandmarkRangeBearingEdge.

            obj = obj@g2o.core.BaseBinaryEdge(2);
        end
        
        function initialEstimate(obj)
            % INITIALESTIMATE Compute the initial estimate of the landmark.
            %
            % Syntax:
            %   obj.initialEstimate();
            %
            % Description:
            %   Compute the initial estimate of the landmark given the
            %   platform pose and observation using the inverse model:
            %
            %   phi = theta + beta
            %   lx = x + r * cos(phi)
            %   ly = y + r * sin(phi)

            % Get the platform state from vertex 1
            x = obj.edgeVertices{1}.x;
            px = x(1);
            py = x(2);
            theta = x(3);
            
            % Get the measurement [r; beta]
            r = obj.z(1);
            beta = obj.z(2);
            
            % Compute the angle in world frame
            phi = theta + beta;
            
            % Compute landmark position using inverse observation model
            lx = px + r * cos(phi);
            ly = py + r * sin(phi);
            
            % Set the estimate for vertex 2 (landmark)
            obj.edgeVertices{2}.setEstimate([lx; ly]);
        end
        
        function computeError(obj)
            % COMPUTEERROR Compute the error for the edge.
            %
            % Syntax:
            %   obj.computeError();
            %
            % Description:
            %   Compute the value of the error:
            %   e = z - h(x, l)
            %
            %   where h(x, l) = [r; beta]:
            %   dx = lx - px; dy = ly - py
            %   r = sqrt(dx^2 + dy^2)
            %   beta = atan2(dy, dx) - theta

            % Get the platform state from vertex 1
            x = obj.edgeVertices{1}.x;
            px = x(1);
            py = x(2);
            theta = x(3);
            
            % Get the landmark state from vertex 2
            l = obj.edgeVertices{2}.x;
            lx = l(1);
            ly = l(2);
            
            % Compute relative position
            dx = lx - px;
            dy = ly - py;
            
            % Compute predicted range
            r_pred = sqrt(dx^2 + dy^2);
            
            % Compute predicted bearing (angle to landmark in robot frame)
            beta_pred = atan2(dy, dx) - theta;
            
            % Compute error: e = z - h(x, l)
            obj.errorZ = obj.z - [r_pred; beta_pred];
            
            % Wrap the bearing error to [-pi, pi]
            obj.errorZ(2) = g2o.stuff.normalize_theta(obj.errorZ(2));
        end
        
        function linearizeOplus(obj)
            % linearizeOplus Compute the Jacobian of the error in the edge.
            %
            % Syntax:
            %   obj.linearizeOplus();
            %
            % Description:
            %   Compute the Jacobian of the error function with respect to
            %   both vertices.
            %
            %   e = z - h(x, l)
            %   J{1} = de/dx = -dh/dx (platform state)
            %   J{2} = de/dl = -dh/dl (landmark state)
            %
            %   h = [r; beta] where:
            %   r = sqrt(dx^2 + dy^2)
            %   beta = atan2(dy, dx) - theta
            %
            %   dh/dx = [dr/dpx,  dr/dpy,  dr/dtheta;
            %            dbeta/dpx, dbeta/dpy, dbeta/dtheta]
            %         = [-dx/r,    -dy/r,    0;
            %            dy/r^2,   -dx/r^2,  -1]
            %
            %   dh/dl = [dr/dlx,    dr/dly;
            %            dbeta/dlx, dbeta/dly]
            %         = [dx/r,     dy/r;
            %            -dy/r^2,  dx/r^2]

            % Get the platform state from vertex 1
            x = obj.edgeVertices{1}.x;
            px = x(1);
            py = x(2);
            
            % Get the landmark state from vertex 2
            l = obj.edgeVertices{2}.x;
            lx = l(1);
            ly = l(2);
            
            % Compute relative position
            dx = lx - px;
            dy = ly - py;
            
            % Compute range and squared range
            r2 = dx^2 + dy^2;
            r = sqrt(r2);
            
            % Numerical stability: avoid division by zero
            if r < 1e-6
                r = 1e-6;
                r2 = r^2;
            end
            
            % Jacobian of h w.r.t. platform state [px, py, theta]
            dh_dx = [-dx/r,   -dy/r,   0;
                      dy/r2,  -dx/r2, -1];
            
            % Jacobian of h w.r.t. landmark state [lx, ly]
            dh_dl = [dx/r,    dy/r;
                    -dy/r2,   dx/r2];
            
            % Error = z - h, so de/d(.) = -dh/d(.)
            obj.J{1} = -dh_dx;
            obj.J{2} = -dh_dl;
        end        
    end
end
