eps = 1e-6;
tol = 0.1;
step = 0.25;
success = 0;
not_finished = [];
failure = [];
dq_max = 3;
ddq_max = 3;
dddq_max = 3;


% Initialize Planner
ltp = LTPlanner(1, 0.001, dq_max, ddq_max, dddq_max);

% Set goal and joint angle
for q_goal = -6:step:7
    q = 0.5;

    % Velocity in Limits
    for dq = -(dq_max-eps):step:(dq_max-eps)

        % Calculate maximal Acceleration to not violate velocity limit
        if dq >= 0
            ddq_lb = -(ddq_max-eps);
            ddq_ub = min(ddq_max-eps, sqrt(2*dddq_max*(dq_max - dq)));
        else
            ddq_lb = max(-(ddq_max-eps), -sqrt(2*dddq_max*(dq_max - abs(dq))));
            ddq_ub = ddq_max;
        end

        % Acceleration in limits
        for ddq = ddq_lb:step:ddq_ub

            % Plan trajectory
            t = ltp.optSwitchTimes(q_goal, q, dq, ddq);
            [q_stop, ~] = ltp.getStopPos(dq, ddq, 1);
            dir = sign(q_goal - (q + q_stop));
            [q_traj, dq_traj, ~] = ltp.getTrajectories(t, dir, q, dq, ddq);

            % Check if goal was reached
            %disp(q_traj(end))
            if abs(q_traj(end) - q_goal) < tol
                success = success + 1;
            else
                if abs(dq_traj(end)) > tol
                    not_finished = [not_finished, [q_goal, q, dq, ddq]'];
                else
                    failure = [failure, [q_goal, q, dq, ddq]'];
                end
            end
        end
    end
end

disp("Success: " + success)
disp("Not finished: " + size(not_finished, 2))
disp("Failure: " + size(failure, 2))

