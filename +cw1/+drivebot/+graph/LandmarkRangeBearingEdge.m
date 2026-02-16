classdef LandmarkRangeBearingEdge < g2o.core.BaseBinaryEdge
    % Range-bearing observation edge (Appendix A.4)

    methods(Access = public)

        function obj = LandmarkRangeBearingEdge()
            obj = obj@g2o.core.BaseBinaryEdge(2);
        end

        function initialEstimate(obj)
            x_p = obj.edgeVertices{1}.estimate();
            x   = x_p(1);
            y   = x_p(2);
            psi = x_p(3);
            r    = obj.z(1);
            beta = obj.z(2);
            phi = psi + beta;
            m_x = x + r * cos(phi);
            m_y = y + r * sin(phi);
            obj.edgeVertices{2}.setEstimate([m_x; m_y]);
        end

        function computeError(obj)
            x_p = obj.edgeVertices{1}.estimate();
            x   = x_p(1);
            y   = x_p(2);
            psi = x_p(3);
            m  = obj.edgeVertices{2}.estimate();
            dx = m(1) - x;
            dy = m(2) - y;
            r_pred    = sqrt(dx^2 + dy^2);
            beta_pred = atan2(dy, dx) - psi;
            obj.errorZ = obj.z - [r_pred; beta_pred];
            obj.errorZ(2) = g2o.stuff.normalize_theta(obj.errorZ(2));
        end

        function linearizeOplus(obj)
            x_p = obj.edgeVertices{1}.estimate();
            x   = x_p(1);
            y   = x_p(2);
            m = obj.edgeVertices{2}.estimate();
            dx = m(1) - x;
            dy = m(2) - y;
            r2 = dx^2 + dy^2;
            r  = sqrt(r2);
            if r < 1e-6
                r  = 1e-6;
                r2 = r^2;
            end
            obj.J{1} = [ dx/r,   dy/r,  0;
                        -dy/r2,  dx/r2, 1];
            obj.J{2} = [-dx/r,   -dy/r;
                         dy/r2,  -dx/r2];
        end
    end
end
