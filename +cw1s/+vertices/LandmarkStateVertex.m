classdef LandmarkStateVertex < g2o.core.BaseVertex
    % LandmarkStateVertex summary of LandmarkStateVertex
    %
    % This vertex stores the state of a 2D landmark: position (x, y).
    % Landmarks are point features in 2D space used for mapping.
    
    properties(Access = public, Constant)
        % Landmark state dimension (x, y)
        NL = 2;
    end
    
    methods(Access = public)
        
        function obj = LandmarkStateVertex()
            % LandmarkStateVertex Constructor
            %
            % Syntax:
            %   obj = LandmarkStateVertex()
            %
            % Description:
            %   Creates a vertex to store the landmark position (x, y).
            
            obj = obj@g2o.core.BaseVertex(2);
        end
        
        % oplus is inherited from BaseVertex - simple addition for
        % Euclidean 2D space
    end
end
