# Unscented Kalman Filter (Term-2 Project-2)
Self-Driving Car Engineer Nanodegree Program

![](https://github.com/emilkaram/SDC-ND-Unscented-Kalman-Filter-Udacity-T2-P2/blob/master/img/0.png)

In this project I utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. 
Calculating the RMSE & NIS values as KPIs to measure performance.

The project code is c++ and uses a Simulator to  demonstrate the UKF.

 I used Linux for coding and run the simulator on Windows 10 using a port forward from the VM (Oracle VM virtualBox). 
 
To compile and run by doing the following from the project top directory.
1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

## Project description:

![](https://github.com/emilkaram/SDC-ND-Unscented-Kalman-Filter-Udacity-T2-P2/blob/master/img/5.png)

# The input data:

/data/obj_pose-laser-radar-synthetic-input.txt

The simulator is using this data file, and feed main.cpp values from it one line at a time.

Each row represents a sensor measurement where the first column indicates if the measurement comes from radar (R) or lidar (L).

For a row containing radar data, the columns are: sensor_type, rho_measured, phi_measured, rhodot_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.

For a row containing lidar data, the columns are: sensor_type, x_measured, y_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.

Whereas radar has three measurements (rho, phi, rhodot), lidar has two measurements (x, y).

I used the measurement values and timestamp in my Unscented Kalman filter algorithm. Groundtruth, which represents the actual path the bicycle took, is for calculating root mean squared error.

The three main steps for programming a Unscented Kalman filter:
initializing 
predicting where our object is going to be after a time step \Delta{t}Δt
updating where our object is based on sensor measurements

Then the prediction and update steps repeat themselves in a loop.

To measure how well my Unscented Kalman filter performs, I calculated root mean squared error comparing the Unscented Kalman filter results with the provided ground truth also calculated the NIS.

and the results met the rubric criteria:

Dataset(1):

![](https://github.com/emilkaram/SDC-ND-Unscented-Kalman-Filter-Udacity-T2-P2/blob/master/img/1.png)

# NIS 
Calculated NIS for Laidar sensor:

 ![](https://github.com/emilkaram/SDC-ND-Unscented-Kalman-Filter-Udacity-T2-P2/blob/master/img/3.png)
 
 
and NIS for Radar sensor:
 ![](https://github.com/emilkaram/SDC-ND-Unscented-Kalman-Filter-Udacity-T2-P2/blob/master/img/4.png)
 
 
# Conclusion:
My algorithm run against Dataset 1 in the simulator which is the same as "data/obj_pose-laser-radar-synthetic-input.txt" in the repository and px, py, vx, and vy RMSEs were less than the values RMSE <= [.09, .10, .40, .30].


