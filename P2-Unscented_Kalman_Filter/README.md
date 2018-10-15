# Unscented Kalman Filter Project

### Compiling  
This project is confirmed to compile on cmake 3.5.1, GNU make 4.1, gcc 5.4.0 in Ubuntu Bash on Windows 10.
   ![compile](https://github.com/na6an/SDCND/blob/master/T2-P2/img/compile.PNG)

****Note*** 
Eigen library in src folder is zipped due to github limitation not able to upload a folder with more than 100 files (See this issue for detail: https://github.com/owncloud/core/issues/19834), nor be able to unzip uploaded zip file.  
Make sure to unzip this after git clone to be able to compile properly.  


### Accuracy  
The result RMSE accuracy is 0.0884, 0.0879, 0.3386, 0.2757 for px, py, vx, and vy on dataset1.
   ![data1](https://github.com/na6an/SDCND/blob/master/T2-P2/img/data1.PNG)

and 0.0843, 0.0774, 0.4059, 0.2814 on dataset2.
   ![data2](https://github.com/na6an/SDCND/blob/master/T2-P2/img/data2.PNG)

### Follows the Correct Algorithm  
*****General Process*****  
The code is written base on kalman filter with standard constant turn rate and velocity magnitude model (CTRV). Unscented kalman filter is applied to non-linear radar update with representative (sigma) points from a Gaussian distribution, while standard kalman filter is used on  lidar update.

*****First Measurement, prediction and update*****  
The code successfully take first measurement, predict and update with new measurement as can be seen in bash.
   ![bash](https://github.com/na6an/SDCND/blob/master/T2-P2/img/bash.PNG)

*****Radar and Laser Measurement*****  
By switching `use_laser_ = true;` or  `use_radar_ = true;` false, either measurement could be selected depends on the sensor type.

*****Parameter Optimization*****  
The initialization was optimized primarily with tuning two process noise standard deviations, std_a_ and std_yawdd_. Value 2.8 was selected after a few trials.  
   ![parameter](https://github.com/na6an/SDCND/blob/master/T2-P2/img/parameter.png)

### Code Efficiency  
No apparent repetition or data reuse is found.

