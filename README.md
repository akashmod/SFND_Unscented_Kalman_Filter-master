# Sensor_Fusion_Unscented_Kalman_Filter
Sensor Fusion UKF Highway Project 

<img src="media/ukf_highway_tracked.gif" width="700" height="400" />

In this project I have implemented an Unscented Kalman Filter to estimate the state of multiple cars on a highway using noisy lidar and radar measurements. Passing the project required obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

The main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ukf_highway



<img src="media/ukf_highway.png" width="700" height="400" />

`main.cpp` uses `highway.h` to create a straight 3 lane highway environment with 3 traffic cars and the main ego car at the center. 
The viewer scene is centered around the ego car and the coordinate system is relative to the ego car as well. The ego car is green while the 
other traffic cars are blue. The traffic cars will be accelerating and altering their steering to change lanes. Each of the traffic car's has
it's own UKF object generated for it, and will update each indidual one during every time step. 

The red spheres above cars represent the (x,y) lidar detection and the purple lines show the radar measurements with the velocity magnitude along the detected angle. The Z axis is not taken into account for tracking, so you are only tracking along the X/Y axis.

---

## Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
 * PCL 1.2

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./ukf_highway`

## Project Overview

### Input to the Filter
The input data consisted of the following columns: Raw Measurements, Timestamp and Ground Truth values of the positions and velocities of the 3 cars to be tracked. The raw measurements are the noisy measurement values taken from Lidar and Radar. They are positions in X and Y axis (px and py) for the lidar measurements and the radius (rho) , azimuth angle (psi) and the velocity (rho_dot) for the radar measurements. The timestamp is in microseconds and denotes the timestamp the data was captured. The Ground Truth values are the actual values of the positions and velocities of the objects in the X and Y axes (px, py, vx, vy) and is to be used for comparison with the estimated values of the same obtained from the Unscented Kalman filter.

### Output from the Filter
The Output of the Unscented Kalman Filter are the estimated values of the state matrix consisting of the positions of the 3 cars in the highway in x and y direction, velocity, yaw and yaw rate of the object. The position estimates are fed into the simulator for marking and are also compared with the ground truth values to evaluate the performance of the filter.

### The Unscented Kalman Filter
The Unscented Kalman Filter is similar to the Extended Kalman Filter in that it has the same inputs and outputs and consists of prediction and update steps. However, instead of linearizing around a fixed point to take care of the non-linearity, the Unscented Kalman Filter uses Sigma Points. The Sigma-Points are generated based on the mean and co-variance. These Sigma points are then passed on for Prediction and Update steps and the consequent means and co-variances are then back calculated from the "Predicted" or "Updated" sigma points. 

#### Prediction step: 
The prediction step predicts the values of the states (px, py, v, yaw and yawd) based on the state transition equations. The state co-variance matrix is updated based on the process noise of the prediction step. Once it receives the mean and co-variance of the state matrix, the sigma points for the states are generated using the equations below. 
  
  X_{k|k} = [ x_{k|k} x_{k|k}+sqrt{(lambda+n_x)*P_{k|k}} x_{k|k}-sqrt{(lambda+n_x)P_{k|k}} ]
                
where lambda is (3- n_x), n_x is the number of states and P is the state co-variance matrix. The number of sigma points are 2*n_x + 1. 

Next, each of these sigma points are passed through the state transition equations to generate the new mean and the co-variance.
                        x' = x + F + v
                        P' = [P 0
                              0 Q]
                          
Where F is the state transition matrix, v is the process noise matrix, Q is the process co-variance matrix which in this case is the variance of longitudnal acceleration and yaw acceleration.

Once these new sigma points are obtained,the new mean and co-variance is obtained from these sigma points by summing them according to their weights.
                
                x_{k+1|k} = sum(w_i * X_{k+1|k,i})

the w_i is the weight matrix which is calculated as:

                    w_0 = lambda/(lambda+n_x)
                    w_1...w_n = 1/(2*lambda)

####Update Step:
The measurements obtained from the Lidar or the Radar are then used to update the predicted states. The lidar update follows the same Kalman Filter equations as before since the measurement matrix of Lidar is linear. Since the Radar measurements are in polar co-ordinates and the states are in cartesian co-ordinates, the measurement matrix for the Radar is non-linear in nature. Hence, an unscented Kalman filter is again used for the radar measurements which involves generating the sigma points and passing them individually through the update steps. The following are the equations for the update step for Lidar measurement update:

				        y = z - H * x
                S = H * P * H' + R
                K = P * H' * S.inverse
                x = x + K * y
                P = (I - K * H) * P
                
For the Radar measurements, the sigma points from the prediction step can be carried forward so we do not have to generate the sigma points again. A measurement prediction is made based on the sigma points and the difference of the actual measurement from the predicted measurement is used to calculate the Kalman and update the co-variance matrix.
                        S = sum(w_i * x_diff * x_diff') + R
                        T = sum(w_i * x_diff * z_diff')   
                        K = T * (S ^ -1)
                        x = x + K * z_diff
                        P = P - K * S * K'
where x_diff is the difference between the sigma point and the states, z_diff is the difference between the actual measurements and the measurement sigma point, R is the noise matrix for the sensor and K is the Kalman gain.  

The UKF.cpp file has the  has the generic flow of the algorithm and the functions for the above mentioned Prediction and Update steps. The Main.cpp file is used to interact with the simulator.

## Evaluation

The RMSE values were found to be less than [.30, .16, 0.95 0.70] as dictated by the project rubric to pass the project. 