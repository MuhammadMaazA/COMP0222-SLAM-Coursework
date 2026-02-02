classdef G2OSLAMSystem < ebe.slam.SLAMSystem
    % G2OSLAMSystem summary of G2OSLAMSystem
    %
    % This class implements a factor graph-based SLAM system using g2o.
    % It maintains a graph of vehicle poses and landmarks, connected by
    % process model edges, GPS edges, and landmark observation edges.
    
    properties(Access = protected)
        
        % The g2o graph optimizer
        graph;
        
        % Current platform vertex
        currentVehicleVertex;
        
        % Map from landmark IDs to vertices
        landmarkIDVertexMap;
        
        % Current odometry and its covariance
        u;
        sigmaU;
        
        % Time of the previous event
        previousTime;
        
        % Step count for internal use
        vehicleVertexCount;
        
        % Performance monitoring
        chi2Store;
        optimizationTimeStore;
        
        % Store of results
        timeStore;
        xStore;
        PStore;
        
        % Configuration
        optimizeOnStep;
        
        % Maximum observations per landmark (for pruning)
        maxObservationsPerLandmark;
        
        % Time window for fixing old poses
        fixOldPosesTimeWindow;
        
        % Whether to unfix all at the end
        unfixAllAtEnd;
    end
    
    methods(Access = public)
        
        function obj = G2OSLAMSystem(config)
            % G2OSLAMSystem Constructor
            %
            % Syntax:
            %   slamSystem = G2OSLAMSystem(config)
            %
            % Description:
            %   Creates a factor graph-based SLAM system using g2o.
            %
            % Inputs:
            %   config - (struct)
            %       Configuration structure
            
            % Call base class constructor
            obj@ebe.slam.SLAMSystem(config);
            
            % Create the graph optimizer
            obj.graph = g2o.core.SparseOptimizer();
            algorithm = g2o.core.LevenbergMarquardtOptimizationAlgorithm();
            obj.graph.setAlgorithm(algorithm);
            
            % Create the landmark map
            obj.landmarkIDVertexMap = configureDictionary("uint32", "any");
            
            % Set default options
            obj.optimizeOnStep = false;
            obj.maxObservationsPerLandmark = inf;
            obj.fixOldPosesTimeWindow = inf;
            obj.unfixAllAtEnd = false;
            
            % Parse config for special options
            if isfield(config, 'optimizeOnStep')
                obj.optimizeOnStep = config.optimizeOnStep;
            end
            if isfield(config, 'maxObservationsPerLandmark')
                obj.maxObservationsPerLandmark = config.maxObservationsPerLandmark;
            end
            if isfield(config, 'fixOldPosesTimeWindow')
                obj.fixOldPosesTimeWindow = config.fixOldPosesTimeWindow;
            end
            if isfield(config, 'unfixAllAtEnd')
                obj.unfixAllAtEnd = config.unfixAllAtEnd;
            end
            
            % Register event handlers
            obj.registerEventHandler('init', @obj.handleInitializationEvent);
            obj.registerEventHandler('null_obs', @obj.handleNoUpdate);
            obj.registerEventHandler('gps', @obj.handleGPSObservationEvent);
            obj.registerEventHandler('slam', @obj.handleSLAMObservationEvent);
            obj.registerEventHandler('odom', @obj.handleUpdateOdometryEvent);
            
            % Set the name
            obj.setName('G2OSLAMSystem');
        end
        
        function setOptimizeOnStep(obj, optimizeOnStep)
            % SETOPTIMIZEONSTEP Set whether to optimize on each step
            obj.optimizeOnStep = optimizeOnStep;
        end
        
        function setMaxObservationsPerLandmark(obj, maxObs)
            % SETMAXOBSERVATIONSPERLANDMARK Set max observations per landmark
            obj.maxObservationsPerLandmark = maxObs;
        end
        
        function setFixOldPosesTimeWindow(obj, timeWindow)
            % SETFIXOLDPOSESTIMEWINDOW Set time window for fixing old poses
            obj.fixOldPosesTimeWindow = timeWindow;
        end
        
        function setUnfixAllAtEnd(obj, unfixAtEnd)
            % SETUNFIXALLATEND Set whether to unfix all vertices at end
            obj.unfixAllAtEnd = unfixAtEnd;
        end
        
        function success = start(obj)
            % START Start the SLAM system
            
            start@ebe.slam.SLAMSystem(obj);
            
            % Clear the graph
            obj.graph = g2o.core.SparseOptimizer();
            algorithm = g2o.core.LevenbergMarquardtOptimizationAlgorithm();
            obj.graph.setAlgorithm(algorithm);
            
            % Clear landmark map
            obj.landmarkIDVertexMap = configureDictionary("uint32", "any");
            
            % Reset counters
            obj.vehicleVertexCount = 0;
            obj.currentVehicleVertex = [];
            obj.previousTime = 0;
            
            % Reset stores
            obj.timeStore = [];
            obj.xStore = zeros(3, 0);
            obj.PStore = zeros(3, 0);
            obj.chi2Store = [];
            obj.optimizationTimeStore = [];
            
            success = true;
        end
        
        function [x, P] = platformEstimate(obj)
            % PLATFORMESTIMATE Get current platform estimate
            
            if isempty(obj.currentVehicleVertex)
                x = zeros(3, 1);
                P = eye(3);
                return;
            end
            
            % Get the estimate
            x = obj.currentVehicleVertex.estimate();
            
            % Get the covariance if needed
            if nargout > 1
                try
                    obj.graph.initializeOptimization();
                    [~, Pcell] = obj.graph.computeMarginals(obj.currentVehicleVertex);
                    P = Pcell{1};
                catch
                    P = eye(3) * 1e-6;
                end
            end
        end
        
        function [T, X, PX] = platformEstimateHistory(obj)
            % PLATFORMESTIMATEHISTORY Get platform estimate history
            T = obj.timeStore;
            X = obj.xStore;
            PX = obj.PStore;
        end
        
        function [m, Pmm, landmarkIds] = landmarkEstimates(obj)
            % LANDMARKESTIMATES Get landmark estimates
            
            landmarkIds = keys(obj.landmarkIDVertexMap);
            numberOfLandmarks = numel(landmarkIds);
            
            m = NaN(2, numberOfLandmarks);
            Pmm = NaN(2, 2, numberOfLandmarks);
            
            if numberOfLandmarks == 0
                return;
            end
            
            % Get all landmark vertices
            landmarkVertices = cell(numberOfLandmarks, 1);
            for l = 1 : numberOfLandmarks
                landmarkVertices{l} = lookup(obj.landmarkIDVertexMap, landmarkIds(l));
                m(:, l) = landmarkVertices{l}.estimate();
            end
            
            % Try to compute covariances
            try
                obj.graph.initializeOptimization();
                [~, Pcells] = obj.graph.computeMarginals(landmarkVertices);
                for l = 1 : numberOfLandmarks
                    Pmm(:, :, l) = Pcells{l};
                end
            catch
                for l = 1 : numberOfLandmarks
                    Pmm(:, :, l) = eye(2) * 1e-6;
                end
            end
        end
        
        function graph = getGraph(obj)
            % GETGRAPH Return the underlying g2o graph
            graph = obj.graph;
        end
        
        function [chi2, optTime] = getPerformanceData(obj)
            % GETPERFORMANCEDATA Get chi2 and optimization time data
            chi2 = obj.chi2Store;
            optTime = obj.optimizationTimeStore;
        end
        
        function optimize(obj, maxIterations)
            % OPTIMIZE Run graph optimization
            if nargin < 2
                maxIterations = 5;
            end
            
            obj.graph.initializeOptimization();
            tic;
            obj.graph.optimize(maxIterations);
            optTime = toc;
            
            % Store performance data
            obj.chi2Store(end+1) = obj.graph.chi2();
            obj.optimizationTimeStore(end+1) = optTime;
        end
        
        function finalOptimization(obj, maxIterations)
            % FINALOPTIMIZATION Run final optimization at end of run
            if nargin < 2
                maxIterations = 10;
            end
            
            % Unfix all vertices if requested
            if obj.unfixAllAtEnd
                vertices = obj.graph.vertices();
                for v = 1:numel(vertices)
                    vertices{v}.setFixed(false);
                end
            end
            
            % Run optimization
            obj.optimize(maxIterations);
        end
    end
    
    methods(Access = protected)
        
        function success = handleNoPrediction(obj)
            % HANDLENOPREDICTION Handle case when no prediction is needed
            success = true;
        end
        
        function success = handleNoUpdate(obj, ~)
            % HANDLENOUPDATE Handle null observation
            success = true;
        end
        
        function success = handlePredictForwards(obj, dT)
            % HANDLEPREDICTFORWARDS Handle prediction step
            %
            % Creates a new vehicle vertex and connects it to the previous
            % one with a PlatformPredictionEdge.
            
            % Check we have a current vertex and odometry
            if isempty(obj.currentVehicleVertex)
                success = true;
                return;
            end
            
            if isempty(obj.u)
                success = true;
                return;
            end
            
            % Create a new vertex for the new pose
            newVertex = cw1s.vertices.VehicleStateVertex();
            obj.graph.addVertex(newVertex);
            
            % Create the prediction edge
            predictionEdge = cw1s.edges.PlatformPredictionEdge();
            predictionEdge.setVertex(1, obj.currentVehicleVertex);
            predictionEdge.setVertex(2, newVertex);
            predictionEdge.setMeasurement(obj.u);
            predictionEdge.setDT(dT);
            predictionEdge.setInformation(inv(obj.sigmaU));
            
            % Initialize the new vertex using the edge
            predictionEdge.initialEstimate();
            
            % Add the edge to the graph
            obj.graph.addEdge(predictionEdge);
            
            % Update the current vertex
            obj.currentVehicleVertex = newVertex;
            obj.vehicleVertexCount = obj.vehicleVertexCount + 1;
            
            % Handle fixing old poses if configured
            if obj.fixOldPosesTimeWindow < inf
                obj.fixOldPoses();
            end
            
            success = true;
        end
        
        function fixOldPoses(obj)
            % FIXOLDPOSES Fix vehicle poses older than time window
            
            currentTime = obj.currentTime;
            vertices = obj.graph.vertices();
            
            for v = 1:numel(vertices)
                vertex = vertices{v};
                % Check if it's a vehicle vertex (3D state)
                if vertex.dimension() == 3
                    % We don't have timestamps on vertices, so we just
                    % fix all but the most recent N vertices
                    % This is a simplification
                end
            end
        end
        
        function success = handleInitializationEvent(obj, event)
            % HANDLEINITIALIZATIONEVENT Handle initialization
            
            % Create the first vehicle vertex
            obj.currentVehicleVertex = cw1s.vertices.VehicleStateVertex();
            obj.currentVehicleVertex.setEstimate(event.data);
            obj.currentVehicleVertex.setFixed(true);  % Fix the initial pose
            
            obj.graph.addVertex(obj.currentVehicleVertex);
            obj.vehicleVertexCount = 1;
            
            obj.initialized = true;
            obj.previousTime = event.time();
            
            success = true;
        end
        
        function success = handleUpdateOdometryEvent(obj, event)
            % HANDLEUPDATEODOMETRYEVENT Update odometry
            
            obj.u = event.data;
            obj.sigmaU = event.covariance;
            
            success = true;
        end
        
        function success = handleGPSObservationEvent(obj, event)
            % HANDLEGPSOBSERVATIONEVENT Handle GPS observation
            
            if isempty(obj.currentVehicleVertex)
                success = true;
                return;
            end
            
            % Create GPS measurement edge
            gpsEdge = cw1s.edges.GPSMeasurementEdge();
            gpsEdge.setVertex(1, obj.currentVehicleVertex);
            gpsEdge.setMeasurement(event.data);
            gpsEdge.setInformation(inv(event.covariance));
            
            obj.graph.addEdge(gpsEdge);
            
            % Optimize if configured to do so on each step
            if obj.optimizeOnStep
                obj.optimize();
            end
            
            success = true;
        end
        
        function success = handleSLAMObservationEvent(obj, event)
            % HANDLESLAMOBSERVATIONEVENT Handle SLAM observations
            
            if isempty(obj.currentVehicleVertex)
                success = true;
                return;
            end
            
            % Get the observation information
            R = event.covariance;
            Omega = inv(R);
            
            % Process each observed landmark
            for i = 1 : numel(event.info)
                landmarkId = event.info(i);
                observation = event.data(:, i);
                
                % Check if this landmark has been seen before
                if isKey(obj.landmarkIDVertexMap, landmarkId)
                    % Existing landmark
                    landmarkVertex = lookup(obj.landmarkIDVertexMap, landmarkId);
                    
                    % Check if we should prune observations
                    if obj.maxObservationsPerLandmark < inf
                        numEdges = landmarkVertex.numberOfEdges();
                        if numEdges >= obj.maxObservationsPerLandmark
                            % Skip this observation
                            continue;
                        end
                    end
                else
                    % New landmark - create a vertex
                    landmarkVertex = cw1s.vertices.LandmarkStateVertex();
                    obj.graph.addVertex(landmarkVertex);
                    obj.landmarkIDVertexMap = insert(obj.landmarkIDVertexMap, ...
                        landmarkId, landmarkVertex);
                end
                
                % Create the observation edge
                obsEdge = cw1s.edges.LandmarkRangeBearingEdge();
                obsEdge.setVertex(1, obj.currentVehicleVertex);
                obsEdge.setVertex(2, landmarkVertex);
                obsEdge.setMeasurement(observation);
                obsEdge.setInformation(Omega);
                
                % Initialize if new landmark
                if ~isKey(obj.landmarkIDVertexMap, landmarkId) || ...
                        any(isnan(landmarkVertex.estimate()))
                    obsEdge.initialEstimate();
                end
                
                obj.graph.addEdge(obsEdge);
            end
            
            % Optimize if configured
            if obj.optimizeOnStep
                obj.optimize();
            end
            
            success = true;
        end
        
        function storeStepResults(obj)
            % STORESTEPRESULTS Store results for this step
            
            if isempty(obj.currentVehicleVertex)
                return;
            end
            
            % Store time and state
            obj.timeStore(obj.stepNumber + 1) = obj.currentTime;
            obj.xStore(:, obj.stepNumber + 1) = obj.currentVehicleVertex.estimate();
            
            % Try to get covariance
            try
                obj.graph.initializeOptimization();
                [~, Pcell] = obj.graph.computeMarginals(obj.currentVehicleVertex);
                obj.PStore(:, obj.stepNumber + 1) = diag(Pcell{1});
            catch
                obj.PStore(:, obj.stepNumber + 1) = ones(3, 1) * 1e-6;
            end
        end
    end
end
