# PID Controller Project
### Compiling  
This project is confirmed to compile on cmake 3.5.1, GNU make 4.1, gcc 5.4.0 in Ubuntu Bash on Windows 10.  
   ![bash](https://github.com/na6an/SDCND/blob/master/T2-P4/image/bash.PNG)  

### P,I,D Components & Tuning process
I began to tune P,I,D parameters in a few fellow peer students suggested in the discussion forum,  
began to increase P until it can handle the sharp turn but not enough to turn too much at the first corner.  
Then, increased D and I until it makes a lap stable. Following are some examples of different parameters.  

PID = (-1,0,0)  
[https://youtu.be/RNQ-9YH_NHk](https://youtu.be/RNQ-9YH_NHk)  
(-0.4,0,0)  
[https://youtu.be/tyWrThhRelc](https://youtu.be/tyWrThhRelc)  
(-0.6,0,0)  
[https://youtu.be/zUI4Cs779o0](https://youtu.be/zUI4Cs779o0)  
(-0.6,0,-5)  
[https://youtu.be/NprE0E6Z-EU](https://youtu.be/NprE0E6Z-EU)

![https://github.com/na6an/SDCND/blob/master/T2-P4/image/pid.png](https://github.com/na6an/SDCND/blob/master/T2-P4/image/pid.png)  
While above is the effect of PID covered in one of the lecture slides,  
following is the overlapped path of PID combinations.  
![https://github.com/na6an/SDCND/blob/master/T2-P4/image/pid-path.png](https://github.com/na6an/SDCND/blob/master/T2-P4/image/pid-path.png)  
Higher value of P helps vehicle to make a sharp turn but also contribute the oscillation more severe.  
Higher value of D reduces oscillation to make driving smoother but up until certain point.  
One thing I observed was, "I" parameter has little effect making the model better.  
Perhaps this is due to the simulation environment, where there is no wheel balance needed.  

I also attempted basic twiddle in the model, but it appears making it work is more time consuming than manual tuning at this point.  
What I adopted instead, is a dynamic speed change because I think tuning only the PID values to control steer is insufficient.  
Basically it slows down when the vehicle turns at the corner or as it deviates more from cte.  
This is the reason simulations above doesn't shoot out of the track even with poor PID tuning.  

### Result  
   [https://youtu.be/rivilUm7wUQ](https://youtu.be/rivilUm7wUQ)  

### Performance 
It appears spec and environment of the computer running the simulation influence vehicle performance.  
The vehicle was able to complete a lap without any issue when I don't run anything on background,  
but running the capturing software took resources away from the simulator and made the vehecle deviate away from the track.  

In such cases, modify `th` variable in following code from `main.cpp`, lower the base speed, which is currently 0.4.
```c++
          double th;
          if (abs(cte) > 1){
            th = 0.4 - cte * cte * speed * abs(angle)/ 1000; //0.4 + pid.UpdateError(cte);
          }else{
            th = 0.4 - abs(cte) * speed / 1000;
          }
```
