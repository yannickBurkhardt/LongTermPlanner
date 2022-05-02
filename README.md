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

When calculating the time phases of maximal, minimal or no jerk as displayed in the image, it must be checked if the limits are reached.  
There exist a total of 8 cases in which phases 2, 4, and 6 could collapse to zero if the maximal acceleration or the maximal velocity are not reached.

For all cases, analytic equations to calculate the switching times could be found.  
However, since some formulas tend to get very long, the roots()-function is used to effiently calculate solutions for some cases.

To simlify computations and reduce runtime, only movements in the positive direction are considered.  
All movements into the negative direction can be mapped into the positive direction to calculate the duration of the phases.  
To find out in which direction the joint must move, the joint angle `q_stop` is calculated if it would stop as quickly as possible.  
The direction of the goal from `q_stop` is the desired direction of movement.


## Time scaling

To achieve that all joints reach the goal at the same time, every joint's movement must be scaled to require as much time as the slowest joint.  
To scale the time required to reach the goal, the sum of time required for all phases must be equal to an externally given time (here: time of the slowest joint): &sum;&Delta;t<sub>i</sub> = t<sub>ext</sub>  
This is archived by calculation of the velocity `v_drive` &le; `v_max` which fulfils the time t<sub>ext</sub>.  
Since the overall time is only affected if v_drive is reached, only 4 of the previously described cases are valid (Phases 2 or 6 may not exist).

However, since v_drive can be smaller than the initial `v_0`, 4 more cases can occur: To reach v_drive, a joint might have to be slowed down.  
This results in phases 1 and 3 of the jerk profile being switched (´mod_jerk_profile´).  
Since we do not have any prior knowledge of how to choose `v_drive`, it is calculated for every case until the time t<sub>ext</sub> is reached with sufficient precision.  

There are very rare scenarios in which reaching `v_drive` slows the overall time more down than desired.  
These cases occur in very special combinations when the joint is already slowing down to reach a goal and the desired time is only a little increased compared to the optimal time.
According to my experiments this is very rare (less than 1 in 1000 cases).  
Due to the infrequence and only little time offset, for these scenarios the optimal time phases can be used without major loss of performance.
