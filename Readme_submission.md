# FCND-Estimation-CPP project Submission #

Submission for the FCND-Controls-CPP project for Philippe Doucette.

***1. Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.***

The calculated standard deviation should correctly capture ~68% of the sensor measurements. Your writeup should describe the method used for determining the standard deviation given the simulated sensor measurements.


![GPS & IMU Standard Deviation](images/Quad_SD.jpg)


***2. Implement a better rate gyro attitude integration scheme in the `UpdateFromIMU()` function.***

The improved integration scheme should result in an attitude estimator of < 0.1 rad for each of the Euler angles for a duration of at least 3 seconds during the simulation. The integration scheme should use quaternions to improve performance over the current simple integration scheme.

***3. Implement all of the elements of the prediction step for the estimator.***

The prediction step should include the state update element (PredictState() function), a correct calculation of the Rgb prime matrix, and a proper update of the state covariance. The acceleration should be accounted for as a command in the calculation of gPrime. The covariance update should follow the classic EKF update equation.

***4. Implement the magnetometer update.***

The update should properly include the magnetometer data into the state. Note that the solution should make sure to correctly measure the angle error between the current state and the magnetometer value (error should be the short way around, not the long way).

***5. Implement the GPS update.***

The estimator should correctly incorporate the GPS information to update the current state estimate.
