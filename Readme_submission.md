# FCND-Estimation-CPP project Submission #

Submission for the FCND-Estimation-CPP project for Philippe Doucette.

### Step 1: Sensor Noise ###
***Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.***

The calculated standard deviation correctly captures ~68% of the sensor measurements. 

The quad simulator was run under scenario `06_SensorNoise` to generate log files `\config\log\Graph1` and `\config\log\Graph2`.  This data was pulled into a spreadsheet and analyzed to generate standard deviation:

![GPS & IMU Standard Deviation](images/Quad_SD.jpg)

These standard deviations were plugged into `config/6_SensorNoise.txt` by updating `MeasuredStdDev_GPSPosXY` and `MeasuredStdDev_AccelXY`.
```
MeasuredStdDev_GPSPosXY = 0.723312024
MeasuredStdDev_AccelXY = 0.4897387239
```

Running the sumulator again, indicates successful capture of 68% of respective measurements:

![SensorNoise](images/6_SensorNoise.png)


### Step 2: Attitude Estimation ###
***Implement a better rate gyro attitude integration scheme in the `UpdateFromIMU()` function.***

The improved integration scheme results in an attitude estimator of < 0.1 rad for each of the Euler angles for a duration of at least 3 seconds during the simulation. 

The integration scheme used quaternions to improve performance over the current simple integration scheme.
```C++
  float yawEst; 
  Quaternion<float> rotationStatePyor;
  Quaternion<float> rotationState;
  
  yawEst = ekfState(6);
  rotationStatePyor = Quaternion<float>:: FromEuler123_RPY(rollEst, pitchEst, yawEst);
  rotationState = rotationStatePyor.IntegrateBodyRate(gyro.getArray(), dtIMU);

  float predictedPitch = rotationState.Pitch();
  float predictedRoll = rotationState.Roll();
  float predictedYaw = rotationState.Yaw();

  ekfState(6) = predictedYaw;
```
![Attitude Estimation](images/7_AttitudeEstimation.png)

### Step 3: Prediction Step ###
***Implement all of the elements of the prediction step for the estimator.***

The prediction step includs the state update element (PredictState() function), a correct calculation of the Rgb prime matrix, and a proper update of the state covariance. The acceleration is accounted for as a command in the calculation of gPrime. The covariance update follows the classic EKF update equation.

PredictState() function was developed as follows:
```C++
  V3F accel_global;
  
  accel_global = attitude.Rotate_BtoI(accel);

  predictedState(0) += predictedState(3) * dt;
  predictedState(1) += predictedState(4) * dt;
  predictedState(2) += predictedState(5) * dt;
  predictedState(3) += accel_global.x * dt;
  predictedState(4) += accel_global.y * dt;
  predictedState(5) += accel_global.z * dt - 9.81f*dt;

  //predictedState(6) = ekfState(6);
  ```
  
8 PredictState tracks
![Predict State](images/8_PredictState.png)


### Step 4: Magnetometer Update ###
***Implement the magnetometer update.***

The update should properly include the magnetometer data into the state. Note that the solution should make sure to correctly measure the angle error between the current state and the magnetometer value (error should be the short way around, not the long way).

### Step 5: Closed Loop + GPS Update ###
***Implement the GPS update.***

The estimator should correctly incorporate the GPS information to update the current state estimate.
