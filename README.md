**Course:** Sensors, Embedded Systems, and Algorithms for Service Robotics (SESASR)
**Institution:** Politecnico di Torino

## Project Overview

This repository hosts the source code and documentation for a robotics project focused on autonomous navigation and state estimation. The core objective was to develop and integrate a robust localization system based on the Extended Kalman Filter (EKF) with a kinematic Path Tracking Controller.
The system enables a mobile robot to estimate its pose (position and orientation) by fusing sensor data and to autonomously follow a reference trajectory with high precision.

## Key Features
### 1. State Estimation (Extended Kalman Filter)
Implemented an Extended Kalman Filter to fuse proprioceptive data (odometry) with exteroceptive sensor measurements.
* **Algorithm:** Deals with the non-linearities of the robot's motion and observation models through Jacobian linearization.
* **Prediction Step:** Propagates the state estimate and error covariance using the velocity motion model.
* **Correction Step:** Updates the belief state by incorporating landmark/feature observations to minimize position drift.

### 2. Path Tracking Controller
Developed a control law to drive the robot along a pre-defined path.
* **Performance:** Ensures smooth trajectory following by minimizing cross-track error and heading error in real-time.

## Theoretical Background

The project relies on the following theoretical pillars:
* **Probabilistic Robotics:** Application of recursive Bayesian filters for state estimation under uncertainty.
* **Kinematic Modeling:** Utilization of differential drive kinematics for motion prediction.
* **Linearization Techniques:** Usage of Taylor series expansion (Jacobians) to adapt the Kalman Filter to non-linear robotic systems.

## Disclaimer

This project was developed as part of the SESASR course requirements. It demonstrates the practical application of algorithms studied in class, specifically focusing on EKF localization and control theory.
