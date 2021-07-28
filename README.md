## PID Controller
Self Driving Car Nanodegree Project

### Project Overview

For this project, a PID controller is used to continuously adjust the steering angle as the car drives around the track. The car needs to be able to stay within the lane through the entire track, and the maximum speed limit is 100 mph. The faster the car drives, the more challenging it becomes to steer smoothly.

### Project Build Instructions

This project uses the Term 3 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2), and the original project repository from Udacity can be found [here](https://github.com/udacity/CarND-PID-Control-Project).

The main program can be built and run by doing the following from the project top directory:

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./path_planning

### PID Overview

The PID controller is a closed feedback loop, continuously calculating the error between the target value and the measured value, and applying a correction to reduce the error. This correction is the sum of three responses: proportional, integral, and derivative:

![image](https://user-images.githubusercontent.com/74683142/127356405-80d0254c-bcbd-487c-a9cd-da6fc16f1bfc.png)

##### Proportional

This controller's response is proportional to the CTE (cross track error), which is the difference between the target value and the measured value. If the error is large, the control output will be proportionally large. As a result, this controller tends to overshoot and the response creates oscillations around the target value.

##### Integral

The integral controller's response is used to correct a continuous error, such as a steering bias due to wheel misalignment. In simulation, the car does not have any bias towards steering to the left or right, so the integral controller is not necessary for this project.

##### Derivative

The derivative controller's response uses the difference between it's previous and current error, which allows it to dampen the oscillations from the proportional response. The greater the current rate of change is, the greater the dampening effect. As a result, this controller slows down how rapidly the error is being corrected. 

### Project Code

The project files are [main.cpp](https://github.com/saulakh/pid-controller/blob/main/src/main.cpp) and [PID.cpp](https://github.com/saulakh/pid-controller/blob/main/src/PID.cpp), located in the ```src``` folder.

##### Update Error

The update error function can be found in _PID.cpp_, and computes the proportional, integral, and derivative errors. The proportional error is the CTE, and the derivative error is the difference between the current CTE and the previous CTE. Since the previous error is equal to the proportional error, I calculated the derivative error before the proportional error updates to the new CTE value. Finally, the integral error is the sum of all of the CTE values.

```
d_error = cte - p_error;
p_error = cte;
i_error += cte;
```

##### Total Error

The total error function can be found in _PID.cpp_, and computes the total sum of the proportional, integral, and derivative errors multiplied by their respective gains:

```return -Kp * p_error - Ki * i_error - Kd * d_error;```

##### Main.cpp

The proportional, integral, and derivative coefficients are tuned in the _main.cpp_ file, and are initialized as Kp = 0.1, Ki = 0.0, and Kd = 2.5. By manually tuning these coefficients, the car was able to drive around the track but it was still swerving quite a bit. To smooth out the driving, I changed the PID coefficients based on speed, since the oscillations were worse at higher speeds. Since the Kp coefficient needed to be lower at higher speeds, I set it inversely proportional to the car's speed. For the Kd value, I started with the initial value and increased it directly proportional to the car's speed to help reduce the oscillations.

```
// Change pid values based on speed
if (speed >= 10) {
  Kp = 1/speed;
  Kd = 2.5 + speed/100;
  cout << "Kp: " << Kp << endl;
  cout << "Kd: " << Kd << endl;
}
```

If the proportional gain was too high, the car would swerve too much and drive off the road. If the derivative gain was too high, or the proportional gain wasn't high enough, the car could be too slow to react to sudden sharp turns and drive off the road in that time.

To resolve this issue, I added a negative throttle for steer values over 0.7, in addition to varying the PID coefficients with speed:

```
// Brake for sharper turns
if (fabs(steer_value) > 0.7) {
  throttle = -0.25;
}
```

Adding the brakes helped the car stay in its lane for sharper turns. Another change I could have implemented was adjusting the throttle based on speed throughout the track, or adding a second PID controller to adjust the car's speed.

### Project Results

After making these changes, the car was able to drive around the track and stay in its lane, driving at approximately 25 - 30 mph.

![pid](https://user-images.githubusercontent.com/74683142/127351770-36c945d8-92c5-4edd-90f1-0e1db2f20e79.gif)

![pid2](https://user-images.githubusercontent.com/74683142/127351980-88410315-1374-46c0-ab4b-a9aa0fb06ca7.gif)
