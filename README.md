# PID Controller Project
---

## Introduction

The goal of this project is to build a lane centering feature for a simulated car using PID controller. Lane centering is based on a cross-track error.  The cross track error is minimized by using PID controller.

## PID Components

P is the proportional term that is directly proportional to the cross track error. The cross track error is the distance from the center of the lane to the center of the car. The proportional gain (Kp) is multiplied with the CTE which means we will steer proportional to the CTE based on the value of the proportional gain.  The proportional component has the most direct effect on the position, however using only the proportional component will cause oscillations by overshooting the target error.

D is the differential term that is the difference between the current and previous values of the error. The differential term is used to minimize the rate of change of the cross track errors over time. In affect, this helps to more smoothly reach the center line.  This is important because the P component acts directly on the CTE and will cause the car to oscillate by overshooting the target error.

I is the integral term that is the accumulated sum of the error. The I component is used on the steering drift.  The integral component is used to hold the total error closer to zero or the center line in the case of driving a car.

### Tuning PID Components

The PID components were tuned  manually. I started with a PD controller only. First I primary goal was to tune the P only component to get the car to move in a straight line. Kp reduced to 0.1 from a starting value of 1.0. Kp of 0.1 was good enough for straight paths but was failing on curves. 

Kd was then tuned for The D component in the same manner as the P component. My final good-enough D value was 3.1 from a starting value of 1.0. At this point the car was driving well enough with just the PD controller, however it wasn't converging properly on curves and turns. 

For Ki, I stared with 1.0 as initial guess but this was far too much and the car became highly unstable. I reduced it by an order of magnitude and eventually found that a good-enough result was about 0.0002. 

At this point with a Kp, Ki, Kd value of 0.1, 0.0002, 3.1 respectively, the car drove fairly smoothly throughout the track but the car seems late into the curves and is a bit erratic at times.  Further filtering of the CTE might help to address this, as well as, the ability to see further down the track --like having map data or longer range sensor data.

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

