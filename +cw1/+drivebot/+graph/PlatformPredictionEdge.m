classdef PlatformPredictionEdge < g2o.core.BaseBinaryEdge
    % Platform prediction edge using odometry (Appendix A.2)

    properties(Access = protected)
        % Time interval for this prediction step
        dT;
    end

    methods(Access = public)

        function obj = PlatformPredictionEdge(dT)
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
            x_k = obj.edgeVertices{1}.estimate();
            x_kp1 = obj.edgeVertices{2}.estimate();
            psi_k = x_k(3);
            c = cos(psi_k);
            s = sin(psi_k);
            invM = (1 / obj.dT) * [c, s, 0; -s, c, 0; 0, 0, 1];
            delta = x_kp1 - x_k;
            dx = delta(1);
            dy = delta(2);
            dInvMdelta_dpsi = (1 / obj.dT) * [-s*dx + c*dy; -c*dx - s*dy; 0];
            obj.J{1} = -invM;
            obj.J{1}(:, 3) = obj.J{1}(:, 3) + dInvMdelta_dpsi;
            obj.J{2} = invM;
        end

    end
end
