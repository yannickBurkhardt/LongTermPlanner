% Initialize parameters
clear all;
tic;
eps = 1e-6;
tol = 0.02;
step = 0.1;
success = 0;
not_finished = [];
failure = [];
v_max = 1;
a_max = 2;
j_max = 15;
Tsample = 0.004;
avg_error = 0;
max_error = 0;

% Initialize planner
ltp = LTPlanner(1, Tsample, v_max, a_max, j_max);

% Set goal and joint angle
for q_goal = -6:step:7
    q_0 = 0.5;

    % Velocity in Limits
    for v_0 = -(v_max-eps):step:(v_max-eps)

        % Calculate maximal acceleration to not violate velocity limit
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
            [t, dir, ~] = ltp.optSwitchTimes(q_goal, q_0, v_0, a_0, 1);
            [q_traj, v_traj, a_traj] = ltp.getTrajectories(t, dir, false, q_0, v_0, a_0);

            % Calculate average and max absolute error
            avg_error = avg_error + abs(q_traj(end) - q_goal);
            if abs(q_traj(end) - q_goal) > max_error
                max_error = abs(q_traj(end) - q_goal);
            end
            
            % Check if goal was reached
            if abs(q_traj(end) - q_goal) < tol
                success = success + 1;
            else
                if abs(v_traj(end)) > tol || abs(a_traj(end)) > tol
                    
                    % Velocity or acceleration at trajectory end not zero
                    not_finished = [not_finished, [q_goal, q_0, v_0, a_0]'];
                else
                    
                    % Goal not reached
                    failure = [failure, [q_goal, q_0, v_0, a_0]'];
                end
            end
        end
    end
end

disp("Success: " + success)
disp("Not finished: " + size(not_finished, 2))
disp("Failure: " + size(failure, 2))
disp("Average error: " + avg_error/(success + size(not_finished, 2) + size(failure, 2)))
disp("Maximal error: " + max_error)

% Throw error if at least one test failed
if size(not_finished, 2) > 0 || size(failure, 2) > 0
    error("Some scenarios were not solved correctly.")
end
toc;
