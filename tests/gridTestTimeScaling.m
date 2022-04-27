% Initialize Parameters
eps = 1e-6;
tol_q = 0.02;
tol_t = 0.1;
step = 0.1;
success = 0;
not_finished = [];
failure = [];
time_error = [];
v_max = 1;
a_max = 2;
j_max = 15;
Tsample = 0.004;
time_increments = [0.05, 0.1, 0.2, 0.5, 1.0, 2.0];

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
            [t, dir, ~] = ltp.optSwitchTimes(q_goal, q_0, v_0, a_0, 1);
            
            % Scale up trajectory
            for i = 1:size(time_increments, 2)
                % Skip if already at goal position
                if(t(end) < tol_q)
                    break;
                end
                
                % Calculate scaled times
                [t_scaled, v_drive, mod_jerk_profile] = ltp.timeScaling(q_goal, q_0, v_0, a_0, dir, 1, t(end) + time_increments(i));
                
                % If scaling failed, assign optimal times
                if(~any(t_scaled))
                    t_scaled = t;
                end
            
                % Calculate trajectories
                [q_traj, v_traj, a_traj] = ltp.getTrajectories(t_scaled, dir, mod_jerk_profile, q_0, v_0, a_0, v_drive);

                % Check if goal was reached in time
                if abs(q_traj(end) - q_goal) < tol_q && abs(t(end) + time_increments(i) - t_scaled(end)) < tol_t
                    success = success + 1;
                else
                    if abs(v_traj(end)) > tol_q || abs(a_traj(end)) > tol_q
                        not_finished = [not_finished, [q_goal, q_0, v_0, a_0]'];
                    elseif abs(q_traj(end) - q_goal) > tol_q
                        failure = [failure, [q_goal, q_0, v_0, a_0]'];
                    else
                        if t_scaled(3) == t_scaled(7)
                            % Goal reached after maximal breaking
                            % (correct behaviour)
                            success = success + 1;
                        else
                            time_error = [time_error, [q_goal, q_0, v_0, a_0]'];
                        end
                    end
                end
            end
        end
    end
end

disp("Success: " + success)
disp("Not finished: " + size(not_finished, 2))
disp("Failure: " + size(failure, 2))
disp("Time error: " + size(time_error, 2))

% Throw error if at least one test failed
if size(not_finished, 2) > 0 || size(failure, 2) > 0 || size(time_error, 2) > 0
    error("Some scenarios were not solved correctly.")
end

