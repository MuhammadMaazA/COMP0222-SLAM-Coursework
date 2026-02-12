classdef PlatformPredictionEdge < g2o.core.BaseBinaryEdge
    % PlatformPredictionEdge Process model edge for GPS-enabled localisation.
    %
    % This binary edge connects two consecutive PlatformStateVertex nodes
    % and encodes the odometry-based process model (Appendix A.2):
    %
    %   x_{k+1} = x_k + M(psi_k) * (u_{k+1} + w_{k+1})
    %
    % where:
    %   x_k     = [x, y, psi]' is the platform state at time k
    %   u_{k+1} = [v_x, v_y, omega]' is the odometry measurement
    %   w_{k+1} ~ N(0, Q) is the process noise
    %   M(psi)  = dT * R(psi) is the scaled rotation matrix (Eq. A.4)
    %
    % The error is computed in body frame so that the information matrix
    % directly corresponds to inv(Q):
    %
    %   e = M^{-1}(psi_k) * (x_{k+1} - x_k) - u

    properties(Access = protected)
        % Time interval for this prediction step
        dT;
    end

    methods(Access = public)

        function obj = PlatformPredictionEdge(dT)
            % Constructor. Creates a 3-dimensional binary edge and stores
            % the prediction time interval dT.
            obj = obj@g2o.core.BaseBinaryEdge(3);
            obj.dT = dT;
        end

        function setDT(obj, dT)
            obj.dT = dT;
        end

        function dT = getDT(obj)
            dT = obj.dT;
        end

        function initialEstimate(obj)
            % INITIALESTIMATE Predict x_{k+1} from x_k using the process model.
            %
            % Implements: x_{k+1} = x_k + M(psi_k) * u
            % where M = dT * [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1]

            x_k = obj.edgeVertices{1}.estimate();
            psi_k = x_k(3);
            u = obj.z;
            c = cos(psi_k);
            s = sin(psi_k);
            M = obj.dT * [c, -s, 0; s, c, 0; 0, 0, 1];
            x_kp1 = x_k + M * u;
            x_kp1(3) = g2o.stuff.normalize_theta(x_kp1(3));
            obj.edgeVertices{2}.setEstimate(x_kp1);
        end

        function computeError(obj)
            % COMPUTEERROR Compute the body-frame error.
            %
            % e = M^{-1}(psi_k) * (x_{k+1} - x_k) - u
            %
            % M^{-1} = (1/dT) * R(psi_k)' rotates the world-frame
            % displacement back into the body frame.

            x_k = obj.edgeVertices{1}.estimate();
            x_kp1 = obj.edgeVertices{2}.estimate();
            psi_k = x_k(3);
            u = obj.z;
            c = cos(psi_k);
            s = sin(psi_k);
            invM = (1 / obj.dT) * [c, s, 0; -s, c, 0; 0, 0, 1];
            obj.errorZ = invM * (x_kp1 - x_k) - u;
            obj.errorZ(3) = g2o.stuff.normalize_theta(obj.errorZ(3));
        end

        function linearizeOplus(obj)
            % LINEARIZEOPLUS Compute the analytical Jacobians of the error.
            %
            % J{2} = de/dx_{k+1} = M^{-1}
            %
            % J{1} = de/dx_k = -M^{-1} + [0 0 d(M^{-1}*delta)/dpsi]
            %   where the third column accounts for the psi-dependence
            %   of the rotation matrix in M^{-1}.

            x_k = obj.edgeVertices{1}.estimate();
            x_kp1 = obj.edgeVertices{2}.estimate();
            psi_k = x_k(3);
            c = cos(psi_k);
            s = sin(psi_k);
            invM = (1 / obj.dT) * [c, s, 0; -s, c, 0; 0, 0, 1];
            delta = x_kp1 - x_k;
            dx = delta(1);
            dy = delta(2);

            % Derivative of M^{-1}*delta with respect to psi_k
            dInvMdelta_dpsi = (1 / obj.dT) * [-s*dx + c*dy; -c*dx - s*dy; 0];

            obj.J{1} = -invM;
            obj.J{1}(:, 3) = obj.J{1}(:, 3) + dInvMdelta_dpsi;
            obj.J{2} = invM;
        end

    end
end
