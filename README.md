# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Goals

In this project the goal is to build a PID controller and tune the PID hyperparameters so the car can safely drive around the track. The simulator will provide the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.


## Reflection

### P component
P stands for proportional, it controls the by steering proportional to the cross-track error(CTE), that means the larger the error the more we turn toward the target trajectory. As we get closer to the trajectory, we'll steer less and less. The following formula is for P component:

`steering = - tau_p * cte`

The problem with the P component is that it will always overshot and never quite attain the correct trajectory, no matter how small the tau_p is. The car's path will oscillate more quickly when tau_p is larger.

### D component
D stands for differential, it will help us avoid the overshoot problem by taking the temporal derivative of the cross-track error(CTE) using the following formula:

` diff_cte = cte - pre_cte
  pre_cte = cte
  steering = - tau_d * diff_cte
`

This means when the car has turned enough to reduce the CTE, it will not just go on trying to reach the target trajectory but will notice that it has already reduced the error, so it makes the car approach its target trajectory gracefully.

### I component
I stands for integral, since PD controller can not solve the systematic bias problem, we can solve this by adding up the cross-track error(CTE) over time. The purpose of I component is to compensate for biases. I component will use the following formula:

`int_cte += cte
 steering = - tau_i * int_cte
`

## Running the Code

### Simulator
You can download the simulator from [here](https://github.com/udacity/self-driving-car-sim/releases).

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

### Dependencies

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
