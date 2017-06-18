# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Performance

The MPC simulated driving reached peak speed of 80 mph, ran at above 40 mph most of the time, and it successfully completed full lap of driving.
This is an feat that it may be even challenging for human driver!
In order to reach high speed driving, there is some slight traffic violation of driving onto 
yellow line or forbidden area occasionally. Reducing the reference speed, can make it full compliant with traffic rules. 

![](./full-lap-peak-80mph.gif)

The kinematic model was used. The Ipopt optimizer was used. 

## Key implementation insights

- Converting from the map's global coordinate system to the local car's coordinate system at the each input-control iteration made the computation much more convenient. 

- The magnitude of various cost (of cte, epsi, difference to the reference speed, etc.) are quite different. I printed out their values to understand the difference, and adjust their coefficients to the total cost. This practice helped to achieve effective search for proper cost function. 

- It seems to me that the cost related to actuators are always zero. It seems that they don't much play much role in the control. 

- The coefficient for the component of the difference to the reference speed may need to be larger than the other components' in order to make sure the car actually move. It might be a better cost to compute the difference to the destination for keeping the car moving. 

- Even with consideration of latency by updating the initial state, the dt in the model should be larger than 0.1 to control the car driving on the road. (It's not clear why. I thought that smaller dt would 
result in more accurate modeling.)

- Larger N (the number of steps in the optimization) helps to provide sufficient space of freedom so that the car can be tolerated to violate some constraints, but eventually and overall reach optimal control. This may result in smoother drive with tolerance not following the reference trajectory to rigidly. 

## Considerations for latency

The initial state read from the simulated car is updated according to the kinematic model with the latency time (100ms). So that the optimizer computed the control solution from the state when 
the actuators can be readily applied to control the car. 

There is some confusion on the value of the steering read from the simulated car. By the [official documentation](https://github.com/udacity/CarND-MPC-Project/blob/master/DATA.md), it's "steering_angle (float) - The current steering angle in radians". While the control fed to the simulated car is a multiple of 0.46332 (in radian equivalent to 25 degrees). I followed the official documentation. I also tried to multiply the steering angle read by 0.46332 in the kinematic equation for the latency update for the psi, car orientation, it didn't make much difference. 

For the update to the speed after latency, the acceleration is harder to obtained. It may be positively related to the throttle value, but be influenced by other factors such as car's mass, the road conditions, etc. So I assumed it to be 3m/s^2 per suggestion from the forum. I also tried with 0 acceleration. With 100ms latency, it doesn't make much difference. 

Although, from modeling perspective, modifying the initial state my the kinematic model would make it more accurate, but from my experiment, it did not make much improvement in the driving control, as long as the time step dt is larger than 0.1 s (the latency is 100ms). This may imply that the latency of 100ms is too small, that the car behavior does not change much is such short time. 

I could estimate the acceleration by recording the speed read from the simulated car, and compute the difference between the speeds of the last control session and current session and divided by the time gap between the two sessions. Due to my observation that the modeling the latency effect may not be critical, thus I decide not to pursuit the effort. 

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

