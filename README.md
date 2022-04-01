# LongTermPlanner

This is a computationally fast and time-optimal trajectory planner.
A trajectory is calculated from an initial state with arbitrary position, velocity, and acceleration to a static goal state: Velocity and acceleration are zero.
The maximal velocities, accelerations and jerks of every joint are limited.  

This planner is useful when a robot controller requires a dense trajectory while external targets are given sparsely (e.g., by a Reinforcement Learning Agent).
It then plans trajectories between these sparse targets.
Generally, a new target should be received before the robot reaches its previous target to allow smooth movements without stopping.
However, if no new targets are received (e.g., in case of connection loss), the robot will safely stop at the target which was received last.  

The LongTermPlanner's trajectories fulfil two criterions:
- Time optimality: The robot must reach the goal state as fast as possible (while obeying its velocity, acceleration, and jerk limits).
- Smoothness: All joints should reach the goal at the same time (which is the optimal time of the slowest joint).  

A trajectory is calculated in a two-step procedure. This is outlined in the following.  

## Time optimality

A general, time optimal trajectory profile of one joint consists of seven phases:
![Time-optimal trajectory](images/profile.svg?raw=true)

When calculating a trajectory, it must be checked if the limits are reached.
For all cases, analytic equations to calculate the switching times could be found.

## Time scaling

To achieve that all joints reach the goal at the same time, every joint's movement must be scaled to require as much time as the slowest joint.
This is archived by a numeric search for the maximal velocities per joint which fulfil this criterion.
After that, the jerk switching times are re-calculated using these velocities.
