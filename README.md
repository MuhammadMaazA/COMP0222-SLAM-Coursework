# COMP0222 Coursework 1 - Factor Graph SLAM

Implementation of a factor graph-based SLAM system using g2o optimization framework in MATLAB.

## Structure

```
+cw1s/
├── +edges/                    # Factor graph edges (YOUR IMPLEMENTATIONS)
│   ├── PlatformPredictionEdge.m   # Q1b: Process model edge
│   ├── GPSMeasurementEdge.m       # GPS observation edge
│   └── LandmarkRangeBearingEdge.m # Q2b: Landmark observation edge
├── +vertices/                 # Factor graph vertices
│   ├── VehicleStateVertex.m       # Vehicle pose (x, y, ψ)
│   └── LandmarkStateVertex.m      # Landmark position (x, y)
├── +drivebot/                 # SLAM systems
│   ├── G2OSLAMSystem.m            # Factor graph SLAM
│   ├── EKFSLAMSystem.m            # EKF-SLAM (for comparison)
│   └── Simulator.m                # Simulation environment
├── config/                    # Configuration files
└── q1_b.m, q1_c.m, ...       # Test scripts

Libraries/                     # g2o and ebe frameworks
Labs/Lab3_EKF_SLAM/           # Reference EKF-SLAM from Lab 3
```

## Usage

1. Open MATLAB and navigate to this folder
2. Run setup:
   ```matlab
   >> setup
   ```
3. Run test scripts:
   ```matlab
   >> cw1s.q1_b   % Test prediction edge
   >> cw1s.q2_b   % Test landmark edge
   ```

## Questions

- **Q1b**: Implement `PlatformPredictionEdge` (initialEstimate, computeError, linearizeOplus)
- **Q1c**: Analyze chi2 and optimization time trends
- **Q2b**: Implement `LandmarkRangeBearingEdge`
- **Q2c**: Compare EKF vs G2O in challenging scenario
- **Q3a**: Evaluate graph pruning strategy
- **Q3b**: Evaluate vertex fixing strategy
