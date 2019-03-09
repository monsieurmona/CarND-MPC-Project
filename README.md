# Model Predictive Control of a Simulated Car
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)
Mario Lüder

Project: CarND-Controls-MPC / Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)

[UpdateEqn]: ./doc/update-equations.png "Update Equations"
[Simulation]: ./doc/simulation.png "Simulation"

## Description
This project implements a solution in C++ for [Model Predictive Control (MPC)](https://en.wikipedia.org/wiki/Model_predictive_control). The goal of this project is drive a simulated car around a track in a Udacity-provided  [simulator](https://github.com/udacity/self-driving-car-sim/releases). The simulator provides the next way points, the car position and speed. The car can be controlled by sending acceleration and steering angle back to the simulator. The solution must take 100 ms latency into account. 

The software packages [CPPAD](https://coin-or.github.io/CppAD/doc/cppad.htm) and [IPOPT](https://projects.coin-or.org/Ipopt) are used to minimize a cost function with its constraints. A short example is given here [ipopt_solve_get_started.cpp.htm](https://www.coin-or.org/CppAD/Doc/ipopt_solve_get_started.cpp.htm)

The costs are a sum of 
* cross track error, 
* orientation error, 
* speed error,
* cost for steering
* cost for acceleration
* cost for steering changes
* cost for acceleration changes     

The constraints are described with a motion model of the form

![Update Equations][UpdateEqn]

The constraints are calculated by the differences of states at t1 and the above formulas, while their results must be zero. 

f(x<sub>t</sub>) is calculated with a third order polynomial f(x<sub>t</sub>) = a + bx<sub>t</sub> + cx<sub>t</sub><sup>2</sup> + dx<sub>t</sub><sup>3</sup> and psides<sub>t</sub> is calculated by (psides<sub>t</sub> = atan(b + 2cx<sub>t</sub> + 3dx<sub>t</sub><sup>2</sup> ) 

The coefficients a,b,c,d where previously calculated by **fitting the third order polynomial** to a given track that was transformed to vehicle coordinate system (track seen from cars perspective) 

The optimizer uses the model to predict a path and actuation commands for a given time interval (**dt = 0.1 s**) and a number of steps **N = 10**.

I took these numbers as suggested by Udacity. Predicting longer than 1 s (dt * N) may not be beneficial anymore as the environment changes. A resolution of 10 may provide a smooth path. 

The **latency** is taken into account by applying the controls a timestep, which are 100ms, later. 
```
if (t > 1)
{
   // use previous actuations (to account for latency)
   a0 = vars[a_startIdx + t - 2]; 
   delta0 = vars[delta_startIdx + t - 2];
}
```

![Simulation][Simulation]
 

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Build with Docker-Compose
The docker-compose can run the project into a container
and exposes the port required by the simulator to run.

1. Clone this repo.
2. Build image: `docker-compose build`
3. Run Container: `docker-compose up`
4. On code changes repeat steps 2 and 3.

## Tips

1. The MPC is recommended to be tested on examples to see if implementation behaves as desired. One possible example
is the vehicle offset of a straight line (reference). If the MPC implementation is correct, it tracks the reference line after some timesteps(not too many).
2. The `lake_track_waypoints.csv` file has waypoints of the lake track. This could fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

