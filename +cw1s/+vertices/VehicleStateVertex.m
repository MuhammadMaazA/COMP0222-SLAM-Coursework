classdef VehicleStateVertex < g2o.core.BaseVertex
    % VehicleStateVertex summary of VehicleStateVertex
    %
    % This vertex stores the state of the vehicle platform: position (x, y)
    % and heading (psi). The heading is wrapped to [-pi, pi] to handle the
    % nonlinear manifold of orientation.
    
    properties(Access = public, Constant)
        % Platform state dimension (x, y, psi)
        NP = 3;
    end
    
    methods(Access = public)
        
        function obj = VehicleStateVertex()
            % VehicleStateVertex Constructor
            %
            % Syntax:
            %   obj = VehicleStateVertex()
            %
            % Description:
            %   Creates a vertex to store the vehicle platform state
            %   (x, y, psi).
            
            obj = obj@g2o.core.BaseVertex(3);
        end
        
        function oplus(obj, update)
            % OPLUS Apply an incremental update to the state estimate.
            %
            % Syntax:
            %   obj.oplus(update);
            %
            % Description:
            %   Applies the update to the state, with special handling for
            %   the heading angle to ensure it stays wrapped in [-pi, pi].
            %
            % Inputs:
            %   update - (3x1 double)
            %       Small perturbation [dx; dy; dpsi]
            
            % Apply the standard addition
            obj.x = obj.x + update;
            
            % Wrap the heading angle to [-pi, pi]
            obj.x(3) = g2o.stuff.normalize_theta(obj.x(3));
        end
    end
end
