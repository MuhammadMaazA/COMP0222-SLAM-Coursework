classdef LandmarkRangeBearingEdge < g2o.core.BaseBinaryEdge
    % LandmarkRangeBearingEdge Range-bearing observation edge for SLAM.
    %
    % This binary edge connects a PlatformStateVertex (pose) and a
    % LandmarkStateVertex (position) and encodes the observation model
    % from Appendix A.4:
    %
    %   z_L = h(x, m) + w_L                                     (A.7)
    %
    % where h(x, m) = [r; beta]:
    %   r    = sqrt((m_x - x)^2 + (m_y - y)^2)       range
    %   beta = atan2(m_y - y, m_x - x) - psi          bearing
    %
    % and w_L ~ N(0, R) is observation noise.
    %
    % The Jacobians are (Eq. A.8):
    %
    %   dh/dx = [-dx/r,  -dy/r,   0;        (2x3, w.r.t. platform)
    %             dy/r2, -dx/r2,  -1]
    %
    %   dh/dm = [dx/r,    dy/r;              (2x2, w.r.t. landmark)
    %           -dy/r2,   dx/r2]
    %
    % where dx = m_x - x, dy = m_y - y, r = sqrt(dx^2+dy^2), r2 = r^2.
    %
    % Note: dh/dm = -dh/dx(:,1:2)   (the 2x2 position block negated).

    methods(Access = public)

        function obj = LandmarkRangeBearingEdge()
            % LandmarkRangeBearingEdge Construct a range-bearing edge.
            %
            % Measurement dimension = 2: [range; bearing].
            % Vertex 1: PlatformStateVertex  [x, y, psi]   (3-DOF)
            % Vertex 2: LandmarkStateVertex  [m_x, m_y]    (2-DOF)

            obj = obj@g2o.core.BaseBinaryEdge(2);
        end

        function initialEstimate(obj)
            % INITIALESTIMATE Initialise the landmark position.
            %
            % Uses the inverse observation model (Appendix A.4):
            %   phi = psi + beta
            %   m   = [x; y] + r * [cos(phi); sin(phi)]
            %
            % This converts the range-bearing measurement back to
            % Cartesian landmark coordinates in the world frame.

            % Platform pose
            x_p = obj.edgeVertices{1}.estimate();
            x   = x_p(1);
            y   = x_p(2);
            psi = x_p(3);

            % Measurement: [range; bearing]
            r    = obj.z(1);
            beta = obj.z(2);

            % Bearing in world frame
            phi = psi + beta;

            % Inverse model: polar -> Cartesian
            m_x = x + r * cos(phi);
            m_y = y + r * sin(phi);

            obj.edgeVertices{2}.setEstimate([m_x; m_y]);
        end

        function computeError(obj)
            % COMPUTEERROR Compute the observation residual.
            %
            %   e = z - h(x, m)
            %
            % where h(x, m) = [r_pred; beta_pred]:
            %   r_pred    = ||m - p||
            %   beta_pred = atan2(dy, dx) - psi
            %
            % The bearing component is wrapped to [-pi, pi].

            % Platform pose
            x_p = obj.edgeVertices{1}.estimate();
            x   = x_p(1);
            y   = x_p(2);
            psi = x_p(3);

            % Landmark position
            m  = obj.edgeVertices{2}.estimate();

            % Relative vector from platform to landmark
            dx = m(1) - x;
            dy = m(2) - y;

            % Predicted observation
            r_pred    = sqrt(dx^2 + dy^2);
            beta_pred = atan2(dy, dx) - psi;

            % Error = measurement - prediction
            obj.errorZ = obj.z - [r_pred; beta_pred];

            % Wrap bearing error to [-pi, pi]
            obj.errorZ(2) = g2o.stuff.normalize_theta(obj.errorZ(2));
        end

        function linearizeOplus(obj)
            % LINEARIZEOPLUS Compute the analytical Jacobians.
            %
            % Since error = z - h(x,m), the Jacobians are:
            %   J{1} = -dh/dx   (2x3)
            %   J{2} = -dh/dm   (2x2)
            %
            % dh/dx and dh/dm are derived from the chain rule applied
            % to h = [sqrt(dx^2+dy^2); atan2(dy,dx) - psi]:
            %
            %   dh/dx = [-dx/r,  -dy/r,   0;
            %             dy/r2, -dx/r2,  -1]
            %
            %   dh/dm = [dx/r,    dy/r;
            %           -dy/r2,   dx/r2]
            %
            % These match Eq. A.8 in the appendix, and are verified
            % against SystemModel.predictSLAMObservation.

            % Platform pose (only need position for Jacobians)
            x_p = obj.edgeVertices{1}.estimate();
            x   = x_p(1);
            y   = x_p(2);

            % Landmark position
            m = obj.edgeVertices{2}.estimate();

            % Relative vector
            dx = m(1) - x;
            dy = m(2) - y;

            % Range and squared range
            r2 = dx^2 + dy^2;
            r  = sqrt(r2);

            % Guard against division by zero
            if r < 1e-6
                r  = 1e-6;
                r2 = r^2;
            end

            % J{1} = -dh/dx   (2x3)
            obj.J{1} = [ dx/r,   dy/r,  0;
                        -dy/r2,  dx/r2, 1];

            % J{2} = -dh/dm   (2x2)
            obj.J{2} = [-dx/r,   -dy/r;
                         dy/r2,  -dx/r2];
        end
    end
end
