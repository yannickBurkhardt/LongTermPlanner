% Initialize parameters
eps = 1e-6;
tol = 0.01;
Tsample = 0.001;
DoF = 6;
v_max = 1*ones(1,DoF);
a_max = 2*ones(1,DoF);
j_max = 15*ones(1,DoF);

% Initialize planner
ltp = LTPlanner(DoF, Tsample, v_max, a_max, j_max);

% Set goal and joint angle
q_0 = 2*3.14*rand(1,DoF) - 3.14;
q_goal = 2*3.14*rand(1,DoF) - 3.14;

% Velocity in limits
v_0 = 2*(v_max-eps).*rand(1,DoF) - (v_max-eps);

% Calculate maximal acceleration to not violate velocity limit
a_lb = zeros(1,DoF);
a_ub = zeros(1,DoF);
for i=1:DoF
    if v_0(i) >= 0
        a_lb(i) = -(a_max(i)-eps);
        a_ub(i) = min(a_max(i)-eps, sqrt(2*j_max(i)*(v_max(i) - v_0(i))));
    else
        a_lb(i) = max(-(a_max(i)-eps), -sqrt(2*j_max(i)*(v_max(i) - abs(v_0(i)))));
        a_ub(i) = a_max(i);
    end
end

% Acceleration in limits
a_0 = (a_ub-a_lb).*rand(1,DoF) + a_lb;

% Plan trajectory
[q_traj, v_traj, a_traj] = ltp.trajectory(q_goal, q_0, v_0, a_0);

% Plot trajectory
plot(q_traj');

% Check if goal was reached
failure = false;
for i=1:DoF
    if abs(q_traj(i,end) - q_goal) > tol
        failure = true;
        disp("Goal not reached: Joint " + i)
        disp("q_goal = " + q_goal(i))
        disp("q(end) = " + q_traj(i,end))
    end
end

if failure
    error("Goal not reached for at least one joint.")
end

