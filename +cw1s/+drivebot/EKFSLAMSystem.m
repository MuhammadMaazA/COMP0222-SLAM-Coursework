classdef EKFSLAMSystem < l3.trianglebot.SLAMSystem
    % EKFSLAMSystem summary of EKFSLAMSystem
    %
    % This class wraps the Lab 3 EKF-SLAM system for use in the coursework.
    % It provides a consistent interface for comparison with the G2O-based
    % SLAM system.
    
    methods(Access = public)
        
        function obj = EKFSLAMSystem(config)
            % EKFSLAMSystem Constructor
            %
            % Syntax:
            %   slamSystem = EKFSLAMSystem(config)
            %
            % Description:
            %   Creates an EKF-based SLAM system.
            %
            % Inputs:
            %   config - (struct)
            %       Configuration structure
            
            obj@l3.trianglebot.SLAMSystem(config);
            obj.setName('EKF-SLAM');
        end
    end
end
