# Extended Kalman Filter Project

### Compiling  
This project is confirmed to compile on cmake 3.5.1, GNU make 4.1, gcc 5.4.0 in Ubuntu Bash on Windows 10.
   ![bash1](https://github.com/na6an/SDCND/blob/master/T2-P1/img/bash1.png)

### Accuracy  
The result RMSE accuracy is 0.095, 0.0875, 0.3696, 0.4624 for px, py, vx, and vy on dataset1.
   ![rmse](https://github.com/na6an/SDCND/blob/master/T2-P1/img/rmse.PNG)

### Follows the Correct Algorithm  
*****General Process*****  
Given the following Kalman Filter formula, algorithm is constructed as general procedure of initialization, prediction, measurement update and evaluation (RMSE).   
x: estimate  
P: uncertainty covariance  
F: state transition matrix  
u: motion vector  
z: measurement  
H: measurement function  
R: measurement noise  
I: identity matrix  

Prediction  
x' = Fx + u  
P’ = FPFT  
Measurement update  
Y = z – Hx  
S = HPHT + R  
K = PHT + S-1  
x' = x + (Ky)  
P’ = (I - KH)P  

*****First Measurement, prediction and update*****  
The code successfully take first measurement, predict and update with new measurement as can be seen in bash.
   ![bash2](https://github.com/na6an/SDCND/blob/master/T2-P1/img/bash2.png)

*****Radar and Laser Measurement*****  
 `KalmanFilter::Update` takes laser measurement while  `KalmanFilter::UpdateEKF` takes non-linear, radar measurement.   

In UpdateEKF, radar angle is normalized with `y_[1] = atan2(sin(y_[1]),cos(y_[1]));` to prevent data skew from anomaly and help reduce RMSE.  
Following is the comparison before and after normalization.  
Before (anomaly data in red box):  
   ![sim1](https://github.com/na6an/SDCND/blob/master/T2-P1/img/sim-bef.PNG)  
After:  
   ![sim2](https://github.com/na6an/SDCND/blob/master/T2-P1/img/sim2.PNG)  

### Code Efficiency  
No apparent repetition or data reuse is found.
