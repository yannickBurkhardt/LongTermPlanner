% Initialize Parameters
eps = 1e-6;
tol = 0.04;
step = 0.25;
success = 0;
not_finished = [];
failure = [];
v_max = 3;
a_max = 3;
j_max = 3;
Tsample = 0.001;

% Initialize Planner
ltp = LTPlanner(1, Tsample, v_max, a_max, j_max);

% Set goal and joint angle
for q_goal = -6:step:7
    q_0 = 0.5;

    % Velocity in Limits
    for v_0 = -(v_max-eps):step:(v_max-eps)

        % Calculate maximal Acceleration to not violate velocity limit
        if v_0 >= 0
            a_lb = -(a_max-eps);
            a_ub = min(a_max-eps, sqrt(2*j_max*(v_max - v_0)));
        else
            a_lb = max(-(a_max-eps), -sqrt(2*j_max*(v_max - abs(v_0))));
            a_ub = a_max;
        end

        % Acceleration in limits
        for a_0 = a_lb:step:a_ub

            % Plan trajectory
            t = ltp.optSwitchTimes(q_goal, q_0, v_0, a_0);
            [q_stop, ~] = ltp.getStopPos(v_0, a_0, 1);
            q_diff = q_goal - (q_0 + q_stop);
            dir = sign(q_diff);
            [q_traj, dq_traj, ~] = ltp.getTrajectories(t, dir, q_0, v_0, a_0);

            % Check if goal was reached
            if abs(q_traj(end) - q_goal) < tol
                success = success + 1;
            else
                if abs(dq_traj(end)) > tol || abs(ddq_traj(end)) > tol
                    not_finished = [not_finished, [q_goal, q_0, v_0, a_0]'];
                else
                    failure = [failure, [q_goal, q_0, v_0, a_0]'];
                end
            end
        end
    end
end

disp("Success: " + success)
disp("Not finished: " + size(not_finished, 2))
disp("Failure: " + size(failure, 2))

