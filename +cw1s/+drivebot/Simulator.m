classdef Simulator < l3.trianglebot.Simulator
    % Simulator summary of Simulator
    %
    % Simulates the scenario for the coursework. This inherits from the
    % Lab 3 simulator to reuse its functionality.
    %
    % The scenario consists of a wheeled robot (DriveBot) driving through
    % a 2D environment populated by landmarks.
    
    methods(Access = public)
        
        function obj = Simulator(config)
            % Simulator Constructor
            %
            % Syntax:
            %   simulator = Simulator(config)
            %
            % Description:
            %   Creates an instance of a Simulator object for the coursework.
            %
            % Inputs:
            %   config - (struct)
            %       The configuration structure
            %
            % Outputs:
            %   simulator - (handle)
            %       An instance of a Simulator
            
            obj@l3.trianglebot.Simulator(config);
        end
    end
end
