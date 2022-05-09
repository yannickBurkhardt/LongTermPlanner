function [q_traj, v_traj, a_traj] = ltpTrajectory(DoF, Tsample, v_max, a_max, j_max, q_goal, q_0, v_0, a_0)
%LTPTRAJECTORY Function to generate sampled trajectory using LTPlanner
% Used to generate C++ Function
ltp = LTPlanner(DoF, Tsample, v_max, a_max, j_max);
[q_traj, v_traj, a_traj] = ltp.trajectory(q_goal, q_0, v_0, a_0);
end

