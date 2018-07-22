# CarND-Controls-MPC
This is my submission for Project 5 of Term 2 of Udacity's Self-Driving Car Engineer Nanodegree Program.

The objective of the project was to use Model Predictive Control (MPC) to implement a controller for actuation of steering angle and accelerator/break of a car in simulator. Here I address the Project rubric points (https://review.udacity.com/#!/rubrics/896/view)


## Rubric points
### Compilation
#### Your code should compile.
Code can be compiled using cmake and make. No changes were necessary for CMake files.
### Implementation
#### The Model
The project uses Kinematic model to describe the vehicle in motion. Kinematic model ignore the forces like tire forces, gravity, drag, inertia, mass and geometry of the vehicle. The model can predict the motion of the vehicle at lower speeds but at higher speeds the model is not reliable.
The state of the vehicle contains following dimensions,
1. x => Position of the vehicle along x axis
2. y => Position of the vehicle along y axis
3. psi => Orientation of the vehicle with respect to x-axis
4. v => Velocity of the vehicle
5. cte => Cross track error, vehicle offset from the reference axis.
6. epsi => Difference between vehicle orientation and reference orientation.
The vehicle can be controlled using following actuations,
1. delta => Steering angle. Constrained to +25 to -25 degree. 
2. a => Throttle controlling the acceleration. -ve values imply deceleration/break.
Following are the update equations used to compute new state of the vehicle, given previous state and actuations.
```
  // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
  // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
  // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
  // v_[t] = v[t-1] + a[t-1] * dt
  // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
  // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
 ```
 where Lf => Distance between steering wheel and the front axle. In this project it is set to 2.67m
 f(x) => 3rd degree polynomical describing the reference path of the vehicle.

#### Timestep Length and Elapsed Duration (N & dt)
Timestep dt was choosen to be 0.1 seconds and N to be 10. Both values were choosen based on the values used in the assignment.

#### Polynomial Fitting and MPC Preprocessing
The simulator provides the waypoints (along with vehicle state and actuations). The waypoints are in map coordinates, transforming them to vehicle coordinate system simplifies the calculation of cte and epsi. This transformation is described in line #110 to #131 of main.cpp

#### Model Predictive Control with Latency
I commented out the sleep in main.cpp and tuned the coefficients in cost function to make sure the car can go around the track atleast once. At the starts of the driving the diff_v component seems to dominate the cost, I could not come up with optimal weights for errors/actuations/actuation-diffs with reference velocity of 100. I initially reduced it to 50 and then tuned the weights, I then slowly tried to increase the speed, I noticed that with values above 65, the vehicle would go off the track after few laps (2 or 3).
I decided to settle at 65.
To address the latency, I initially tried to simply send the actuation values computed for time t + delay_in_time_steps.
But this did not work out as the model had used incorrect actuations for updating states.
I then decided to apply the delay to the input state. To compute the next state I used the same Kinematic model equations as above.
I added following new variables to class MPC for this purpose.
```  
  // Delay in seconds
  double delay_sec;
  // Previous steering angle actuation
  double pre_delta;
  // Previous throttle actuation
  double pre_a;
```
### Simulation
#### The vehicle must successfully drive a lap around the track.
The vehicle drives around the track atleast few laps (atleast 5 laps). I noticed that the vehicle does go off the track after sometimes, may be the weights used in cost functions are not yet optimal.


---

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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
