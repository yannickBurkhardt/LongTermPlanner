function [q_traj_fixed_len, dq_traj_fixed_len, ddq_traj_fixed_len] = lttPlannerFcn(q_goal, q, dq, ddq)

%% Parameter and Variable initialization
% Maximal jerk, acceleration and velocity for every joint
dddq_max = 4;
ddq_max = 2;
dq_max = 2;

% Degrees of freedom of the robot
DoF = 1;

% Sample time
Tsample = 0.001;

%Output length in samples
num_samples = 10000;

% Factor by which dq is scaled down per iteration (tunable):
% Has to be in (0;1)
% Smaller values -> faster runtime
% Larger values -> smoother trajectories (joints are more likely to arrive at the same time)
dq_reduction = 0.9;

% Parameters for calculation
dir = zeros(DoF,1); % Direction of the goal (only +1 or -1)
t_rel = zeros(DoF,7); % Time that is required for one jerk phase
t_rel_prev = zeros(DoF,7); % Times from previous iteration
t = zeros(DoF,7); % Absolute time that is required to reach the end of current jerk phase
stop = zeros(DoF,1); % Exception if a goal cannot be reached -> Stop this joint asap (boolean)
slowest_i = 0; % Number of slowest joint
t_max = inf; % Time to reach goal for slowest joint
traj_len = zeros(DoF,1); % Length of trajectory in samples

% Save current values
ddq_backup = ddq;
dq_backup = dq;
q_backup = q;
                      
%% Analyse input data
t_rel_reset_ddq = zeros(DoF,1);
for i=1:DoF
    % Check input velocity and acceleration
    if(abs(ddq(i)) > ddq_max(i) || abs(dq(i)) > dq_max(i))
        error("Input velocity or acceleration exceed limits.")
    elseif (abs(dq(i) + 1/2*ddq(i)*abs(ddq(i))/dddq_max(i)) > dq_max(i))
        error("Input velocity will exceed limits because acceleration is too high.")
    end
    
    % Stop joint for position equal to goal
    if(q_goal(i) == q(i))
        stop(i) = 1;
        continue
    end
    
    % Calculate direction of movement
    dir(i) = sign(q_goal(i) - q(i));
    
    % If current ddq is opposite to goal direction, first adust ddq to zero
    if((q_goal(i) - q(i))*ddq(i) < 0)
        t_rel_reset_ddq(i) = abs(ddq(i))/dddq_max(i);
        dq_ = dq(i) + ddq(i) * t_rel_reset_ddq(i) + dir(i)*1/2*dddq_max(i)*t_rel_reset_ddq(i)^2; % do not overwrite yet
        q(i) = q(i) + dq(i)*t_rel_reset_ddq(i) + 1/2*ddq(i)*t_rel_reset_ddq(i)^2 + dir(i)*1/6*dddq_max(i)*t_rel_reset_ddq(i)^3;
        dq(i) = dq_;
        ddq(i) = 0;
    end
    
    % If goal is in negative direction, change dq and ddq
    if(dir(i) < 0)
        dq(i) = -dq(i);
        ddq(i) = -ddq(i);
    end
end
    
%% Calculate times of jerk switch
% Execute this twice:
% 1. Calculate min. time required per joint to reach goal state
% 2. Iteratively reduce max. dq per joint until all joints need the same time to reach the goal
for iter=1:2
    for i=1:DoF
        
        % Skip this joint if current joint has to be stopped or is the
        % slowest joint
        if(stop(i) || i == slowest_i)
            continue
        end
        
        % Reduction loop of dq_max (if necessary)
        reduce_ddq_max = 1;
        %%
        while(reduce_ddq_max)
            reduce_ddq_max = 0;

            % j = +- j_max (const) -> linear a
            t_rel(i,1) = (ddq_max(i) - ddq(i))/dddq_max(i); %CHECK
            t_rel(i,3) = ddq_max(i)/dddq_max(i);
            t_rel(i,5) = t_rel(i,3);
            t_rel(i,7) = t_rel(i,3);

            % a = +- a_max (const) -> linear v
            % works but only if all times > 0
            t_rel(i,2) = (dq_max(i) - dq(i) - 1/2*t_rel(i,1)*ddq(i))/ddq_max(i) - 1/2*(t_rel(i,1) + t_rel(i,3));
            t_rel(i,6) = dq_max(i)/ddq_max(i) - 1/2*(t_rel(i,5) + t_rel(i,7));

            % Check if max acceleration cannot be reached
            if(t_rel(i,2) < 0)
                % Check if root is positive
                % (should always be)
                if(dddq_max(i)*(dq_max(i) - dq(i)) + 1/2*ddq(i)^2 > 0)
                    t_rel(i,3) = sqrt(dddq_max(i)*(dq_max(i) - dq(i)) + 1/2*ddq(i)^2)/dddq_max(i);
                    t_rel(i,1) = t_rel(i,3) - ddq(i)/dddq_max(i);
                    t_rel(i,2) = 0;
                else
                    % Stop movement
                    stop(i) = 1;
                    break
                end
            end
            if(t_rel(i,6)<0)
                % Check if root is positive
                % (should always be)
                if(dq_max(i)/dddq_max(i) > 0)
                    t_rel(i,5) = sqrt(dq_max(i)/dddq_max(i));
                    t_rel(i,7) = t_rel(i,5);
                    t_rel(i,6) = 0;
                else
                    % Stop movement
                    stop(i) = 1;
                    break
                end
            end

            % v = v_max (const) -> linear q
            q_part1 = dq(i)*(t_rel(i,1) + t_rel(i,2) + t_rel(i,3)) + ddq(i)*(1/2*t_rel(i,1)^2 + t_rel(i,1)*(t_rel(i,2) + t_rel(i,3)) + 1/2*t_rel(i,3)^2) + dddq_max(i)*(1/6*t_rel(i,1)^3 + 1/2*t_rel(i,1)^2*(t_rel(i,2) + t_rel(i,3)) - 1/6*t_rel(i,3)^3 + 1/2*t_rel(i,1)*t_rel(i,3)^2) + ddq_max(i)*(1/2*t_rel(i,2)^2 + t_rel(i,2)*t_rel(i,3));
            q_part2 = dddq_max(i)*(1/6*t_rel(i,7)^3 + 1/2*t_rel(i,7)^2*(t_rel(i,6) + t_rel(i,5)) - 1/6*t_rel(i,5)^3 + 1/2*t_rel(i,7)*t_rel(i,5)^2) + ddq_max(i)*(1/2*t_rel(i,6)^2 + t_rel(i,6)*t_rel(i,5));
            t_rel(i,4) = (abs(q_goal(i) - q(i)) - q_part1 - q_part2)/dq_max(i);

            % Check if max velocity cannot be reached
            if(t_rel(i,4) < 0)
                % Formula calculated with matlab
                % Check if root is positive
                root = (dddq_max(i)^2*t_rel(i,1)^4)/2 - (dddq_max(i)^2*t_rel(i,3)^4)/4 + (dddq_max(i)^2*t_rel(i,3)^2*t_rel(i,5)^2)/2 - (dddq_max(i)^2*t_rel(i,5)^4)/4 + (dddq_max(i)^2*t_rel(i,7)^4)/2 + 2*dddq_max(i)*ddq(i)*t_rel(i,1)^3 - (2*dddq_max(i)*ddq_max(i)*t_rel(i,1)^3)/3 - 2*dddq_max(i)*ddq_max(i)*t_rel(i,1)*t_rel(i,3)^2 + (2*dddq_max(i)*ddq_max(i)*t_rel(i,3)^3)/3 + (2*dddq_max(i)*ddq_max(i)*t_rel(i,5)^3)/3 - 2*dddq_max(i)*ddq_max(i)*t_rel(i,5)^2*t_rel(i,7) - (2*dddq_max(i)*ddq_max(i)*t_rel(i,7)^3)/3 + 2*dddq_max(i)*dq(i)*t_rel(i,1)^2 + 2*ddq(i)^2*t_rel(i,1)^2 - 2*ddq(i)*ddq_max(i)*t_rel(i,1)^2 - 2*ddq(i)*ddq_max(i)*t_rel(i,3)^2 + 4*ddq(i)*dq(i)*t_rel(i,1) + 2*ddq_max(i)^2*t_rel(i,3)^2 + 2*ddq_max(i)^2*t_rel(i,5)^2 - 4*ddq_max(i)*dq(i)*t_rel(i,1) + 4*abs(q(i) - q_goal(i))*ddq_max(i) + 2*dq(i)^2;
                if(0 < root)
                    t_rel(i,6) = -(4*ddq_max(i)*t_rel(i,5) - 2*root^(1/2) + dddq_max(i)*t_rel(i,3)^2 - dddq_max(i)*t_rel(i,5)^2 + 2*dddq_max(i)*t_rel(i,7)^2)/(4*ddq_max(i));
                    t_rel(i,2) = (-dq(i) - ddq(i)*t_rel(i,1) - 1/2*dddq_max(i)*t_rel(i,1)^2 + 1/2*dddq_max(i)*t_rel(i,3)^2 + 1/2*dddq_max(i)*t_rel(i,7)^2 - 1/2*dddq_max(i)*t_rel(i,5)^2)/ddq_max(i) - t_rel(i,3) + t_rel(i,6) + t_rel(i,5);
                    t_rel(i,4) = 0;
                else
                    % Stop movement
                    stop(i) = 1;
                    break
                end
                % Check if max velocity and max acceleration cannot be reached
                if(t_rel(i,6) < 0 || t_rel(i,2) < 0) 
                    % Equations not solvable, reduce a_max until solution is found
                    reduce_ddq_max = 1;
                    dq_max(i) = dq_max(i) * dq_reduction;
                    % Stop if dq_max is reduced too far
                    if(dq_max(i) < abs(dq(i)))
                        % Stop movement
                        stop(i) = 1;
                        break
                    end
                end
            end

            % Add ddq reset to first time
            t_rel(i,1) = t_rel(i,1) + t_rel_reset_ddq(i);
            
            % Stop if any time is negative
            for j=1:7
                if(t_rel(i,j) < 0)
                    stop(i) = 1;
                    break
                end
            end

            % If joint is slower than slowest joint,
            % keep times from last iteration for this joint
            if(sum(t_rel(i,:)) < t_max)
                
                % Calculate absolute times for jerk switches
                t(i,:) = cumsum(t_rel(i,:));
                
                % Reduce dq_max for next iteration
                dq_max(i) = dq_max(i) * dq_reduction;
                reduce_ddq_max = 1;
                
                % Save last t_rel
                t_rel_prev(i,:) = t_rel(i,:);

                % Leave loop here in first run (the minimal times per joint are calculated)
                if(iter == 1)
                    break
                end

                % Leave loop and reset to last times if dq_max is reduced too far
                if(dq_max(i) < abs(dq(i)) || abs(dq(i) + 1/2*ddq(i)*abs(ddq(i))/dddq_max(i)) > dq_max(i))
                    break
                end
            end
        end
        
        % Reset to times from previous iteration and reset exceptions
        if(iter == 2)
            t_rel(i,:) = t_rel_prev(i,:);
            stop(i) = 0;
        end
    end

    % Find slowest joint
    if(iter == 1)
        t_max = 0;
        for i=1:DoF
            if(t(i,7) > t_max)
                t_max = t(i,7);
                slowest_i = i;
            end
        end
    end
end

%% Scale jerks of all joints so they require the same time to reach goal
t_diff = t_max - t(:,7);
j_factor = ones(DoF,1);

for i=1:DoF
    t_factor = 2*t_diff(i)/t_rel(i,5) + 1;
    t_rel_5_scaled = t_factor*t_rel(i,5);
    t_rel_6_scaled = t_rel(i,6) + (1 - t_factor)*t_rel(i,5);
    t_rel_7_scaled = t_factor*t_rel(i,7);
    t_rel_4_scaled = t_rel(i,4) + 1/2*(1 - t_factor)*t_rel(i,5);
    
    % If t4 is negative, calculate factor such that t4 = 0
    if(t_rel_4_scaled < 0)
        t_factor = 1 + 2*t_rel(i,4)/t_rel(i,5);
        t_rel_5_scaled = t_factor*t_rel(i,5);
        t_rel_6_scaled = t_rel(i,6) + (1 - t_factor)*t_rel(i,5);
        t_rel_7_scaled = t_factor*t_rel(i,7);
        t_rel_4_scaled = 0;
    end
    % Check if ddq_max is not reached
    if(t_rel_6_scaled < 0)
        % Calculate new factor
        t_factor = (t_diff(i) + t_rel(i,5) + 1/2*t_rel(i,6))^2/(ddq_max(i)/dddq_max(i)*t_rel(i,6) + t_rel(i,5)^2);
        
        % Check if root is positive (should always be, but just to make sure)
        if(t_factor*ddq_max(i)/dddq_max(i)*t_rel(i,6) + t_rel(i,5)^2 > 0)
            t_rel_5_scaled = sqrt(t_factor*(ddq_max(i)/dddq_max(i)*t_rel(i,6) + t_rel(i,5)^2));
            t_rel_7_scaled = t_rel_5_scaled;
            t_rel_6_scaled = 0;
            t_rel_4_scaled = t_rel(i,4) + t_rel(i,5) + 1/2*t_rel(i,6) - t_rel_5_scaled;
            
            % If t4 is negative, calculate factor such that t4 = 0
            if(t_rel_4_scaled < 0)
                % Calculate new factor
                t_factor = (t_rel(i,4) + t_rel(i,5) + 1/2*t_rel(i,6))^2/(ddq_max/dddq_max*t_rel(i,6) + t_rel(i,5)^2);
                
                % Check if root is positive (should always be, but just to make sure)
                if(t_factor*ddq_max(i)/dddq_max(i)*t_rel(i,6) + t_rel(i,5)^2 > 0)
                    t_rel_5_scaled = sqrt(t_factor*(ddq_max(i)/dddq_max(i)*t_rel(i,6) + t_rel(i,5)^2));
                    t_rel_7_scaled = t_rel_5_scaled;
                    t_rel_6_scaled = 0;
                    t_rel_4_scaled = 0;
                else
                    % Keep previous solution
                    continue
                end
            end
        else
            % Keep previous solution
            continue
        end
    end
    t_rel(i,4) = t_rel_4_scaled;
    t_rel(i,5) = t_rel_5_scaled;
    t_rel(i,6) = t_rel_6_scaled;
    t_rel(i,7) = t_rel_7_scaled;
    
    % Set factor for jerks
    j_factor(i) = 1/t_factor;
    
    % Calculate absolute times
    t(i,:) = cumsum(t_rel(i,:));
end

% Re-apply changed values
ddq = ddq_backup;
dq = dq_backup;
q = q_backup;

%% Check if exception at one joint
% (Exceptions will occur if the joint will overshoot the goal position
% because it cannot break fast enough. Therefore, the jount will be stopped
% as fast as possible)
for i=1:DoF
    if(stop(i))
        % Set direction opposite to maximal velocity to be reached
        dir(i) = -sign(dq(i) + 1/2*ddq(i)*abs(ddq(i))/dddq_max(i));
        
        % If current ddq and d are in the same direction, first bring ddq
        % to zero
        ddq_backup(i) = ddq(i);
        dq_backup(i) = dq(i);
        if(dq(i)*ddq(i) > 0)
            t_rel_reset_ddq(i) = abs(ddq(i))/dddq_max(i);
            dq(i) = dq(i) + ddq(i) * t_rel_reset_ddq(i) - sign(dq(i))*1/2*dddq_max(i)*t_rel_reset_ddq(i)^2;
            ddq(i) = 0;
        end
        
        % If dq is in positive, change dq and ddq
        if(dir(i) < 0)
            ddq(i) = -ddq(i);
            dq(i) = -dq(i);
        end
        
        % Stop joint asap
        t(i,:) = zeros(1,size(t,2));
        t_rel(i,1) = (ddq_max(i) - ddq(i))/dddq_max(i);
        t_rel(i,3) = ddq_max(i)/dddq_max(i);
        t_rel(i,2) = (- dq(i) - 1/2*t_rel(i,1)*ddq(i))/ddq_max(i) - 1/2*(t_rel(i,1) + t_rel(i,3));
        if(t_rel(i,2) < 0)
            % Check if root is positive, otherwise switch times
            root = dddq_max(i)*dq(i) - 1/2*ddq(i)*abs(ddq(i));
            if(root > 0)
                t_rel(i,1) = sqrt(root)/dddq_max(i);
                t_rel(i,3) = t_rel(i,1) - ddq(i)/dddq_max(i);
            else
                t_rel(i,3) = sqrt(-root)/dddq_max(i);
                t_rel(i,1) = t_rel(i,3) - ddq(i)/dddq_max(i);
            end
            t_rel(i,2) = 0;
        end

        % Sum up relative times
        t(i,1:3) = cumsum(t_rel(i,1:3)) + t_rel_reset_ddq(i);
        t(i,4:7) = t(i,3);
        
        % Check if this takes longer than slowest joint
        if(t(i,3) > t_max)
            t_max = t(i,3);
        end
        
        % Length of trajectory in samples
        traj_len(i) = ceil(t(i,7)/Tsample) + 1;
        
        % Reset ddq
        ddq(i) = ddq_backup(i);
        dq(i) = dq_backup(i);
    end
end
t_rel

%% Calculate Trajectories
% Length of trajectory in samples
for i=1:DoF
    traj_len(i) = ceil(t(i,7)/Tsample) + 1;
end

% Init trajectories with fixed length
ddq_traj_fixed_len = zeros(DoF,num_samples);
dq_traj_fixed_len = zeros(DoF,num_samples);
q_traj_fixed_len = zeros(DoF,num_samples);

for i=1:DoF
    % Calculate jerks, accelerations, velocities and positions
    dddq_traj = zeros(1,traj_len(i));
    
    % Calculate jerks
    sampled_t = ones(1,7); % Jerk switch times in samples
    for j=1:7
        sampled_t(j) = floor(t(i,j)/Tsample) + 1;
    end

    
    % Calculate jerk trajectory
    dddq_traj(1:sampled_t(1)) = dir(i) * dddq_max(i);
    dddq_traj(sampled_t(1):sampled_t(2)) = 0;
    dddq_traj((sampled_t(2)+1):sampled_t(3)) = -dir(i) * dddq_max(i);
    dddq_traj(sampled_t(3):sampled_t(4)) = 0;
    dddq_traj((sampled_t(4)+1):sampled_t(5)) = -dir(i) * j_factor(i) * dddq_max(i);
    dddq_traj(sampled_t(5):sampled_t(6)) = 0;
    dddq_traj((sampled_t(6)+1):sampled_t(7)) = dir(i) * j_factor(i) * dddq_max(i);
    
    % Calculate accelerations
    ddq_traj = Tsample * cumsum(dddq_traj) + ddq(i);
    
    % Calculate velocities
    dq_traj = Tsample * cumsum(ddq_traj) + dq(i);
    
    % Calculate positions
    q_traj = Tsample * cumsum(dq_traj) + q(i);

    % Save trajectories in arrays with fixed lengh
    q_traj_fixed_len(i,:) = q_traj(traj_len(i))*ones(1,num_samples);
    ddq_traj_fixed_len(i,1:traj_len(i)) = ddq_traj;
    dq_traj_fixed_len(i,1:traj_len(i)) = dq_traj;
    q_traj_fixed_len(i,1:traj_len(i)) = q_traj;
end

end
