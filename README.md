# LongTermPlanner

This is a computationally fast and time-optimal trajectory planner.
A trajectory is calculated from an initial state with arbitrary position, velocity, and acceleration to a static goal state: Velocity and acceleration are zero.
The maximal velocities, accelerations and jerks of every joint are limited. 

The LongTermPlanner is written in MATLAB and optimized for usage with Simulink Real-Time.  
Thanks to [JakobThumm](https://github.com/JakobThumm), there is now also a C++ version of this project. The respective source code can be found in the `src` and `include` folders.

This planner is useful when a robot controller requires a dense trajectory while external targets are given sparsely (e.g., by a Reinforcement Learning Agent).
It then plans trajectories between these sparse targets.
Generally, a new target should be received before the robot reaches its previous target to allow smooth movements without stopping.
However, if no new targets are received (e.g., in case of connection loss), the robot will safely stop at the target which was received last.  

The LongTermPlanner's trajectories fulfil two criterions:
- Time optimality: The robot must reach the goal state as fast as possible (while obeying its velocity, acceleration, and jerk limits).
- Smoothness: All joints should reach the goal at the same time (which is the optimal time of the slowest joint).  

A trajectory is calculated in a two-step procedure. This is outlined in the following.  

## How to use

### MATLAB

Firstly, the LongTermPlanner must be initialized with the desired properties:  
```
ltp = LTPlanner(DoF, Tsample, v_max, a_max, j_max);
```

To generate a sampled trajectory, its `trajectory` function can be used.
As transfer parameters, it requires the joint's goal positions `q_goal` and their initial angles `q_0`, velocities `v_0` and accelerations `a_0`.
It returns the sampled trajectory information of acceleration, velocity, and angle for all joints at every time step:  
```
[q_traj, v_traj, a_traj] = ltp.trajectory(q_goal, q_0, v_0, a_0);
```

### C++

The installation requires `gcc`, `c++>=17`, and `Eigen3` version 3.4 (download it here: https://eigen.tuxfamily.org/index.php?title=Main_Page).
Set the path to your eigen3 installation to this env variable, e.g.,
```
export EIGEN3_INCLUDE_DIR="/usr/include/eigen3/eigen-3.4.0"
```

Use `cmake` to install:
```
mkdir build && cd build
cmake ..
make -j8
```
Run the unit tests with (`gtest` is required)
```
ctest --output-on-failure
```
Usage:
Initialize the `LongTermPlanner` with the planning limits 
```
int dof = ...;
double t_sample = ...;
std::vector<double> q_min = {...};
std::vector<double> q_max = {...};
std::vector<double> v_max = {...};
std::vector<double> a_max = {...};
std::vector<double> j_max = {...};
LongTermPlanner ltp(dof, t_sample, q_min, q_max, v_max, a_max, j_max);
```
Run the planning for a desired goal
```
Trajectory traj;
std::vector<double> q_goal = {...};
std::vector<double> q_0 = {...};
std::vector<double> v_0 = {...};
std::vector<double> a_0 = {...};
bool success = bool ltp.planTrajectory(q_goal, q_0, v_0, a_0, traj);
```
Your output `Trajectory` is a struct of the form
```
struct Trajectory {        
  int dof;
  double t_sample;
  int length;
  std::vector<std::vector<double>> q;
  std::vector<std::vector<double>> v;
  std::vector<std::vector<double>> a;
  std::vector<std::vector<double>> j;
};
```
## Time optimality

A general, time optimal trajectory profile of one joint consists of seven phases:  
<img src="images/profile.svg?raw=true" alt="Time-optimal Trajectory" width="600"/>

When calculating the time phases of maximal, minimal or no jerk (as displayed in the image), it must be checked if the limits are reached.
There exist a total of 8 cases in which phases 2, 4, and 6 could collapse to zero if the maximal acceleration or the maximal velocity are not reached.

For all cases, analytic equations to calculate the switching times could be found.
However, since some formulas tend to get very long, the roots()-function is used to efficiently calculate solutions for some cases.

To simplify computations and reduce runtime, only movements in the positive direction are considered.
All movements into the negative direction can be mapped into the positive direction to calculate the duration of the phases.  
To find out in which direction the joint must move, the joint angle `q_stop` is calculated if it would stop as quickly as possible.
The direction of the goal from `q_stop` is the desired direction of movement.

## Time scaling

To achieve that all joints reach the goal at the same time, every joint's movement must be scaled to require as much time as the slowest joint.
To scale the time required to reach the goal, the sum of time required for all phases must be equal to an externally given time (here: time of the slowest joint): &sum;&Delta;t<sub>i</sub> = t<sub>ext</sub>  
This is archived by calculation of the velocity `v_drive` &le; `v_max` which fulfils the time t<sub>ext</sub>.  
Since the overall time is only affected if v_drive is reached, only 4 of the previously described cases are valid (Phases 2 or 6 may not exist).

However, since v_drive can be smaller than the initial `v_0`, 4 more cases can occur: To reach v_drive, a joint might have to be slowed down.
This results in phases 1 and 3 of the jerk profile being switched ("modified jerk profile").
Since we do not have any prior knowledge of how to choose `v_drive`, it is calculated for every case until the time t<sub>ext</sub> is reached with sufficient precision.  
A trajectory using the modified jerk profile to reach v_drive in phase 4 is displayed in the following image:  
<img src="images/modifiedJerkProfile.svg?raw=true" alt="Modified Jerk Profile" width="600"/>

There are very rare scenarios in which reaching `v_drive` slows the overall time more down than desired.
These cases occur in very special combinations when the joint is already slowing down to reach a goal and the desired time is only a little increased compared to the optimal time.
According to my experiments, this case appears very rarely (less than 1 out of 1000 cases).
Due to the infrequence and only little time offset, for these scenarios the optimal time phases can be used without major loss of performance.

## Performance

In this section, accuracy and runtime are analysed.

### Accuracy

The following results are obtained by simulating all realistic scenarios using a grid search with a fixed step size of 0.1.  
`j_max` = 15 rad/ s^3  
`a_max` = 2 rad/ s^2  
`v_max` = 1 rad/ s  

With these limits, the average absolute error at the goal position was found to be 0.003 rad.  
The worst-case error remains below 0.015 rad.

The results can be reproduced using `tests/gridTestTimeScaling.m`

### Runtime

For runtime analysis, trajectory planning for a 6 DoF-robot was simulated in MATLAB 2020a running on an Intel Core i5-6267U processor (2.9 GHz, 4MB L3 Cache).
For a full planning procedure with a sample time of 4 ms, the following runtimes were obtained:  
Average: 0.48 ms  
Worst Case: 2.29 ms  

For this, `tests/randomConfiguration` was used looping the trajectory generation.
The worst case was simulated by forcing the planner to execute every possible calculation (no breaking if a valid solution was found).

## Examples

In the following plot, a trajectory for a 6-DoF robot calculated by the LongTermPlanner is displayed:  
![Time-optimal trajectory](images/exampleTrajectory.svg?raw=true)

A sampled trajectory contains information about the acceleration, velocity and joint angle for each joint and at each time step.  
It can be seen that all joints reach the goal at the same time with velocity and acceleration reaching zero.
The slowest joint (Joint 1) reaches the maximal velocity of 1 rad/ s.
`v_drive` for the other joints is calculated such that the goal is reached as the same time as for Joint 1.
Note that Joint 6 is slowed down to reach `v_drive`, so the modified jerk profile is used.
