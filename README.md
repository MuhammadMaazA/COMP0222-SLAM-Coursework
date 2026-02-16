# COMP0222 Coursework 1

MATLAB implementation of graph-based SLAM using the g2o optimisation framework, developed as part of the COMP0222 module at UCL. The system compares **EKF-SLAM** and **G2O factor graph SLAM** on a simulated robot navigating a landmark environment with GPS and odometry.

## How to Run

1. Open MATLAB and navigate to this project folder.
2. Run the setup script:
   ```matlab
   setup
   ```
3. For optimal results, clear any cached class definitions before each run:
   ```matlab
   clear classes
   ```
4. Run any question script:
   ```matlab
   cw1.q1_b    % Q1b: GPS-only localisation
   cw1.q1_c    % Q1c: Chi2 and optimisation time analysis
   cw1.q2_b    % Q2b: Full SLAM with landmarks
   cw1.q2_c    % Q2c: Challenging scenario comparison
   cw1.q3_a    % Q3a: Edge pruning scalability strategy
   cw1.q3_b    % Q3b: Vertex fixing scalability strategy
   ```

## Note on Results

Due to the stochastic nature of the simulator (random noise on odometry and observations), **each run will produce slightly different results**. The outputs presented in the report were selected from the best representative runs.
