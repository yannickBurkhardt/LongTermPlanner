v_max = 2;
a_max = 2;
j_max = 4;
Tsample = 0.001;

% Initialize planner
ltp = LTPlanner(1, Tsample, v_max, a_max, j_max);
traj = ltp.trajectory(1.1, 1.0, 0.0, 1e-8);