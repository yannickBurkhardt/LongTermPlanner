classdef LTPlanner < handle
    %LTPLANNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Degrees of freedom of the robot
        DoF;

        % Sample time
        Tsample;

        %Output length in samples
        num_samples = 10000;
        
        % Maximal jerk, acceleration and velocity for every joint
        j_max;
        a_max;
        v_max;
    end
    
    methods
        function obj = LTPlanner(varargin)
            %LTPLANNER Construct an instance of this class
            if nargin>2
                obj.v_max = varargin{3};
                obj.a_max = varargin{4};
                obj.j_max = varargin{5};
            end
            obj.DoF = varargin{1};
            obj.Tsample = varargin{2};
        end
        
        function obj = setLimits(obj, v_max, a_max, j_max)
            %SETLIMITS Set maximal jerk, acceleration and velocity for
            %every joint
            obj.j_max = j_max;
            obj.a_max = a_max;
            obj.v_max = v_max;
        end
        
        function [q_traj, v_traj, a_traj] = trajectory(obj, q_goal, q_0, v_0, a_0)
            t_opt = zeros(obj.DoF, 7);            % Optimal jerk switching times
            t_scaled = zeros(obj.DoF, 7);         % Scaled jerk switching times
            dir = zeros(obj.DoF, 1);              % Direction of movement
            mod_jerk_profile = false(obj.DoF, 1); % Boolean to choose jerk profile
            
            % Bring inputs into correct format
            if size(q_goal, 2) > 1
                q_goal = q_goal';
            end
            if size(q_0, 2) > 1
                q_0 = q_0';
            end
            if size(v_0, 2) > 1
                v_0 = v_0';
            end
            if size(a_0, 2) > 1
                a_0 = a_0';
            end
            
            %% Find slowest joint
            % Calculate time-optimal profiles
            for joint = 1:obj.DoF
                [t_opt(joint,:), dir(joint), ~] = optSwitchTimes(obj, q_goal(joint), q_0(joint), v_0(joint), a_0(joint), joint);
            end
            
            % Save minimum time and joint
            [t_required, slowest_joint] = max(t_opt(:,end));
            
            
            %% Scale other joints to require same time
            v_drive = obj.v_max;
            for joint = 1:obj.DoF
                if joint == slowest_joint
                    continue;
                end
                [t_scaled(joint,:), v_drive(joint), mod_jerk_profile(joint)] = timeScaling(obj, q_goal(joint), q_0(joint), v_0(joint), a_0(joint), dir(joint), joint, t_required);
            end
            
            % If no solution was found, use optimal time
            for joint = 1:obj.DoF
                if(~any(t_scaled(joint,:)))
                    t_scaled(joint,:) = t_opt(joint,:);
                end
            end
            
            %% Calculate sampled trajectories
            [q_traj, v_traj, a_traj] = getTrajectories(obj, t_scaled, dir, mod_jerk_profile, q_0, v_0, a_0, v_drive);
        end
            
        
        function checkInputs(obj, v_0, a_0, joint)
            % CHECKINPUTS Check input velocity and acceleration
            if(abs(v_0) > obj.v_max(joint))
                error("Input velocity exceeds limits.")
            end
            if(abs(a_0) > obj.a_max(joint))
                error("Input acceleration exceeds limits.")
            end
            if (abs(v_0 + 1/2*a_0*abs(a_0)/obj.j_max(joint)) > obj.v_max(joint))
                error("Input velocity will exceed limits because acceleration is too high.")
            end
        end
        
        function [t, dir, mod_jerk_profile] = optSwitchTimes(varargin)
            %OPTSWITCHTIMES Calculate time-optimal jerk swtiches
            
            % Apply transfer parameters
            obj = varargin{1};
            q_goal = varargin{2};
            q_0 = varargin{3};
            v_0 = varargin{4};
            a_0 = varargin{5};
            joint = varargin{6};
                
            % v_drive describes the constant velocity in phase 4.
            % For time-optimal planning, this is v_max.
            % For time-scaling, v_drive is calculated in timeScaling
            if nargin > 6
                v_drive = varargin{7};
            else
                v_drive = obj.v_max(joint);
            end
            
            % Parameters for calculation
            t_rel = zeros(1,7); % Time that is required for one jerk phase
            mod_jerk_profile = false; % Use the standard jerk profile if not changed during calculations
            eps = 4e-3;
            
            % Check if inputs are in limits
            checkInputs(obj, v_0, a_0, joint);
            
            % Calculate direction of movement
            [q_stop, t_rel(1:3), dir] = optBraking(obj, v_0, a_0, joint);
            q_diff = q_goal - (q_0 + q_stop);
            if (abs(q_diff) < eps)
                % Skip rest if that fulfils scenario
                t = cumsum(t_rel);
                return;
            end
            dir = sign(q_diff);

            % If goal is in negative direction, map to pos. direction
            if(dir < 0)
                v_0 = -v_0;
                a_0 = -a_0;
            end

            %% Calculate min. time required per joint to reach goal state
            
            % Check if slowing down is necessary to satisfy v_drive
            q_brake = 0;
            if(v_0 + 1/2*a_0*abs(a_0)/obj.j_max(joint) > v_drive)
                mod_jerk_profile = true;
                
                % Get Joint Position after braking and required times
                [q_brake, t_rel(1:3)] = optBraking(obj, v_0 - v_drive, a_0, joint);
            else

                % Constant max/ min jerk (Phase 1, 3)
                t_rel(1) = (obj.a_max(joint) - a_0)/obj.j_max(joint);
                t_rel(3) = obj.a_max(joint)/obj.j_max(joint);

                % Constant acceleration (Phase 2)
                t_rel(2) = (v_drive - v_0 - 1/2*t_rel(1)*a_0)/obj.a_max(joint) - 1/2*(t_rel(1) + t_rel(3));

                % Check if phase 2 does not exist
                % (max acceleration cannot be reached)
                if(t_rel(2) < -eps)
                    % Check if root is positive
                    root = obj.j_max(joint)*(v_drive - v_0) + 1/2*a_0^2;
                    if(root > 0)
                        t_rel(3) = sqrt(root)/obj.j_max(joint);
                        t_rel(1) = t_rel(3) - a_0/obj.j_max(joint);
                        t_rel(2) = 0;
                    else
                        % This should hould never occur, only for safety
                        t = zeros(1,7);
                        return;
                    end
                end
            end

            % Constant max/ min jerk (Phase 5, 7)
            t_rel(5) = obj.a_max(joint)/obj.j_max(joint);
            t_rel(7) = t_rel(5);

            % Constant acceleration (Phase 6)
            t_rel(6) = v_drive/obj.a_max(joint) - 1/2*(t_rel(5) + t_rel(7));

            % Check if phase 6 does not exist
            % (max acceleration cannot be reached)
            if(t_rel(6) < -eps)
                % Check if root is positive
                root = v_drive/obj.j_max(joint);
                if(root > 0)
                    t_rel(5) = sqrt(root);
                    t_rel(7) = t_rel(5);
                    t_rel(6) = 0;
                else
                    % This should hould never occur, only for safety
                    t = zeros(1,7);
                    return;
                end
            end

            % Constant velocity (Phase 4)
            if(mod_jerk_profile)
                % Braking to satisfy v_drive
                q_part1 = q_brake + v_drive*(t_rel(1) + t_rel(2) + t_rel(3));
            else
                % Acceleration to reach v_drive
                q_part1 = v_0*(t_rel(1) + t_rel(2) + t_rel(3)) + a_0*(1/2*t_rel(1)^2 + t_rel(1)*(t_rel(2) + t_rel(3)) + 1/2*t_rel(3)^2) + obj.j_max(joint)*(1/6*t_rel(1)^3 + 1/2*t_rel(1)^2*(t_rel(2) + t_rel(3)) - 1/6*t_rel(3)^3 + 1/2*t_rel(1)*t_rel(3)^2) + obj.a_max(joint)*(1/2*t_rel(2)^2 + t_rel(2)*t_rel(3));
            end
            q_part2 = obj.j_max(joint)*(1/6*t_rel(7)^3 + 1/2*t_rel(7)^2*(t_rel(6) + t_rel(5)) - 1/6*t_rel(5)^3 + 1/2*t_rel(7)*t_rel(5)^2) + obj.a_max(joint)*(1/2*t_rel(6)^2 + t_rel(6)*t_rel(5));
            t_rel(4) = ((q_goal - q_0)*dir - q_part1 - q_part2)/v_drive;

            % Check if phase 4 does not exist
            % (max velocity cannot be reached)
            if(t_rel(4) < -eps)

                if(mod_jerk_profile)
                    % This case is not valid
                    % (is handled in timeScaling function)
                    t = zeros(1,7);
                    return;
                end

                % Calculate times if max velocity is not reached 
                root = (obj.j_max(joint)^2*t_rel(1)^4)/2 - (obj.j_max(joint)^2*t_rel(3)^4)/4 + (obj.j_max(joint)^2*t_rel(3)^2*t_rel(5)^2)/2 - (obj.j_max(joint)^2*t_rel(5)^4)/4 + (obj.j_max(joint)^2*t_rel(7)^4)/2 + 2*obj.j_max(joint)*a_0*t_rel(1)^3 - (2*obj.j_max(joint)*obj.a_max(joint)*t_rel(1)^3)/3 - 2*obj.j_max(joint)*obj.a_max(joint)*t_rel(1)*t_rel(3)^2 + (2*obj.j_max(joint)*obj.a_max(joint)*t_rel(3)^3)/3 + (2*obj.j_max(joint)*obj.a_max(joint)*t_rel(5)^3)/3 - 2*obj.j_max(joint)*obj.a_max(joint)*t_rel(5)^2*t_rel(7) - (2*obj.j_max(joint)*obj.a_max(joint)*t_rel(7)^3)/3 + 2*obj.j_max(joint)*v_0*t_rel(1)^2 + 2*a_0^2*t_rel(1)^2 - 2*a_0*obj.a_max(joint)*t_rel(1)^2 - 2*a_0*obj.a_max(joint)*t_rel(3)^2 + 4*a_0*v_0*t_rel(1) + 2*obj.a_max(joint)^2*t_rel(3)^2 + 2*obj.a_max(joint)^2*t_rel(5)^2 - 4*obj.a_max(joint)*v_0*t_rel(1) + 4*dir*(q_goal - q_0)*obj.a_max(joint) + 2*v_0^2;
                if(root > 0)
                    t_rel(6) = -(4*obj.a_max(joint)*t_rel(5) - 2*root^(1/2) + obj.j_max(joint)*t_rel(3)^2 - obj.j_max(joint)*t_rel(5)^2 + 2*obj.j_max(joint)*t_rel(7)^2)/(4*obj.a_max(joint));
                    t_rel(2) = (-v_0 - a_0*t_rel(1) - 1/2*obj.j_max(joint)*t_rel(1)^2 + 1/2*obj.j_max(joint)*t_rel(3)^2 + 1/2*obj.j_max(joint)*t_rel(7)^2 - 1/2*obj.j_max(joint)*t_rel(5)^2)/obj.a_max(joint) - t_rel(3) + t_rel(6) + t_rel(5);
                    t_rel(4) = 0;
                else
                    % This should hould never occur, only for safety
                    t = zeros(1,7);
                    return;
                end

                % Check if phase 2 and/ or phase 6 does not exist 
                % (max velocity and max acceleration cannot be reached)
                if(t_rel(6) < -eps || t_rel(2) < -eps)
                    root = roots([12, 0, (-24*a_0^2 + 48*obj.j_max(joint)*v_0), (48*dir*obj.j_max(joint)^2*q_0 - 48*dir*obj.j_max(joint)^2*q_goal + 16*a_0^3 - 48*a_0*obj.j_max(joint)*v_0), -3*a_0^4 + 12*a_0^2*obj.j_max(joint)*v_0 - 12*obj.j_max(joint)^2*v_0^2]);

                    % Choose non-complex, positive solution
                    root = root((abs(imag(root)) < eps));
                    root = root(root >= 0);
                    t_rel(1) = (2*root(1)^2 - 4*a_0*root(1) + a_0^2 - 2*v_0*obj.j_max(joint))/(4*obj.j_max(joint)*root(1));

                    % Calculate other switch times
                    t_rel(7) = sqrt(4*obj.j_max(joint)^2*t_rel(1)^2 + 8*a_0*obj.j_max(joint)*t_rel(1) + 2*a_0^2 + 4*obj.j_max(joint)*v_0)/(2*obj.j_max(joint));
                    t_rel(5) = a_0/obj.j_max(joint) + t_rel(1) + t_rel(7);
                    t_rel(2) = 0;
                    t_rel(6) = 0;

                    % Check if a_max is exceeded (Phase 2 exists)
                    if(a_0 + t_rel(1)*obj.j_max(joint) > obj.a_max(joint))
                        t_rel(1) = (obj.a_max(joint) - a_0) / obj.j_max(joint);
                        t_rel(7) = 1/obj.j_max(joint) * (obj.a_max(joint)/2 + sqrt(9*obj.a_max(joint)^2 + 6*sqrt(-12*obj.a_max(joint)*obj.j_max(joint)^3*t_rel(1)^3 + 9*a_0^2*obj.j_max(joint)^2*t_rel(1)^2 - 18*a_0*obj.a_max(joint)*obj.j_max(joint)^2*t_rel(1)^2 + 9*obj.a_max(joint)^2*obj.j_max(joint)^2*t_rel(1)^2 + 36*a_0*obj.j_max(joint)^2*t_rel(1)*v_0 - 72*obj.a_max(joint)*dir*obj.j_max(joint)^2*q_0 + 72*obj.a_max(joint)*dir*obj.j_max(joint)^2*q_goal - 36*obj.a_max(joint)*obj.j_max(joint)^2*t_rel(1)*v_0 + 3*obj.a_max(joint)^4 + 36*obj.j_max(joint)^2*v_0^2))/6 - obj.a_max(joint));
                        t_rel(5) = t_rel(7) + obj.a_max(joint)/obj.j_max(joint);
                        t_rel(2) = -(-obj.j_max(joint)*t_rel(5)^2 - 2*obj.j_max(joint)*t_rel(5)*t_rel(7) + obj.j_max(joint)*t_rel(7)^2 + a_0*t_rel(1) + obj.a_max(joint)*t_rel(1) + 2*obj.a_max(joint)*t_rel(5) + 2*obj.a_max(joint)*t_rel(7) + 2*v_0)/(2*obj.a_max(joint));
                        t_rel(6) = 0;
                    end

                    % Check if -a_max is exceeded (Phase 6 exists)
                    if(t_rel(7)*obj.j_max(joint) > obj.a_max(joint))
                        t_rel(7) = obj.a_max(joint)/obj.j_max(joint);
                        root = roots([12, - 24*obj.a_max(joint), (-12*a_0^2 + 12*obj.a_max(joint)^2 + 24*obj.j_max(joint)*v_0), 0, 24*dir*obj.j_max(joint)^2*q_0*obj.a_max(joint) - 24*dir*obj.j_max(joint)^2*q_goal*obj.a_max(joint) + 3*a_0^4 + 8*a_0^3*obj.a_max(joint) + 6*a_0^2*obj.a_max(joint)^2 - 12*a_0^2*obj.j_max(joint)*v_0 - 24*a_0*obj.j_max(joint)*v_0*obj.a_max(joint) - 12*obj.a_max(joint)^2*obj.j_max(joint)*v_0 + 12*obj.j_max(joint)^2*v_0^2]);

                        % Choose non-complex, positive solution
                        root = root((abs(imag(root)) < eps));
                        root = root(root >= 0);
                        t_rel(1) = (root - a_0 - obj.a_max(joint))/obj.j_max(joint);

                        % Calculate other switch times
                        t_rel(5) = (a_0 + obj.a_max(joint))/obj.j_max(joint) + t_rel(1);
                        t_rel(6) = (obj.j_max(joint)^2*t_rel(1)^2 + 2*obj.j_max(joint)^2*t_rel(1)*t_rel(5) - obj.j_max(joint)^2*t_rel(5)^2 + 2*a_0*obj.j_max(joint)*t_rel(1) + 2*a_0*obj.j_max(joint)*t_rel(5) - obj.a_max(joint)^2 + 2*obj.j_max(joint)*v_0)/(2*obj.j_max(joint)*obj.a_max(joint));
                        t_rel(2) = 0;
                    end

                    % All other times are 0
                    t_rel(3) = 0;
                    t_rel(4) = 0;
                end
            end

            % Safety checks
            if any(t_rel < -eps)
                % No numeric inaccuracy
                t_rel = zeros(1,7);
            end

            if any(abs(imag(t_rel)) > eps)
                % No numeric inaccuracy
                t_rel = zeros(1,7);
            end
            
            % Small numeric errors are set to 0
            t_rel = max(0, real(t_rel));

            % Calculate absolute times for jerk switches
            t = cumsum(t_rel);
        end
        
        function [t, v_drive, mod_jerk_profile] = timeScaling(obj, q_goal, q_0, v_0, a_0, dir, joint, t_required)
            % TIMESCALING % Calculate switching times to fulfil a given
            % time by adjusting the maximally reached velocity
            
            % Parameters for calculation
            tol = 0.1;

            % If goal is in negative direction, map to pos. direction
            if(dir < 0)
                v_0 = -v_0;
                a_0 = -a_0;
            end

            %% Calculate required v_drive to reach goal at given time
            % Standard jerk profile: Phases 2 and 6 exist
            v_drive = (obj.a_max(joint)*obj.j_max(joint)*t_required/2 - a_0^2/4 + a_0*obj.a_max(joint)/2 - obj.a_max(joint)^2/2 + v_0*obj.j_max(joint)/2 - sqrt(36*obj.a_max(joint)^2*obj.j_max(joint)^2*t_required^2 - 36*a_0^2*obj.a_max(joint)*obj.j_max(joint)*t_required + 72*a_0*obj.a_max(joint)^2*obj.j_max(joint)*t_required - 72*obj.a_max(joint)^3*obj.j_max(joint)*t_required + 144*obj.a_max(joint)*dir*obj.j_max(joint)^2*q_0 - 144*obj.a_max(joint)*dir*obj.j_max(joint)^2*q_goal + 72*obj.a_max(joint)*obj.j_max(joint)^2*v_0*t_required - 9*a_0^4 + 12*a_0^3*obj.a_max(joint) + 36*a_0^2*obj.a_max(joint)^2 + 36*a_0^2*obj.j_max(joint)*v_0 - 72*a_0*obj.a_max(joint)^3 - 72*a_0*obj.a_max(joint)*obj.j_max(joint)*v_0 + 36*obj.a_max(joint)^4 - 36*obj.j_max(joint)^2*v_0^2)/12)/obj.j_max(joint);

            % Check if v_drive is real and positive
            if ~imag(v_drive) && v_drive > 0
                [t, ~, mod_jerk_profile] = optSwitchTimes(obj, q_goal, q_0, dir*v_0, dir*a_0, joint, v_drive);

                % Check time constraint was fulfilled
                if t_required - t(end) < tol && t_required - t(end) > -tol/10
                    return;
                end
            end

            % Modified jerk profile: Phases 2 and 6 exist
            v_drive = -(dir*(q_0 - q_goal) - obj.j_max(joint)*((a_0 + obj.a_max(joint))^3/(6*obj.j_max(joint)^3) - obj.a_max(joint)^3/(6*obj.j_max(joint)^3) + (obj.a_max(joint)^2*(a_0 + obj.a_max(joint)))/(2*obj.j_max(joint)^3) + ((a_0 + obj.a_max(joint))^2*((v_0 + (a_0*(a_0 - obj.a_max(joint)))/(2*obj.j_max(joint)))/obj.a_max(joint) + obj.a_max(joint)/(2*obj.j_max(joint)) + (a_0 - obj.a_max(joint))/(2*obj.j_max(joint))))/(2*obj.j_max(joint)^2)) + a_0*((a_0 + obj.a_max(joint))^2/(2*obj.j_max(joint)^2) + obj.a_max(joint)^2/(2*obj.j_max(joint)^2) + ((a_0 + obj.a_max(joint))*((v_0 + (a_0*(a_0 - obj.a_max(joint)))/(2*obj.j_max(joint)))/obj.a_max(joint) + obj.a_max(joint)/(2*obj.j_max(joint)) + (a_0 - obj.a_max(joint))/(2*obj.j_max(joint))))/obj.j_max(joint)) - obj.a_max(joint)*(((v_0 + (a_0*(a_0 - obj.a_max(joint)))/(2*obj.j_max(joint)))/obj.a_max(joint) - obj.a_max(joint)/(2*obj.j_max(joint)) + (a_0 - obj.a_max(joint))/(2*obj.j_max(joint)))^2/2 + (obj.a_max(joint)*((v_0 + (a_0*(a_0 - obj.a_max(joint)))/(2*obj.j_max(joint)))/obj.a_max(joint) - obj.a_max(joint)/(2*obj.j_max(joint)) + (a_0 - obj.a_max(joint))/(2*obj.j_max(joint))))/obj.j_max(joint)) + v_0*((v_0 + (a_0*(a_0 - obj.a_max(joint)))/(2*obj.j_max(joint)))/obj.a_max(joint) + (a_0 + obj.a_max(joint))/obj.j_max(joint) + obj.a_max(joint)/(2*obj.j_max(joint)) + (a_0 - obj.a_max(joint))/(2*obj.j_max(joint))))/(obj.a_max(joint)/(2*obj.j_max(joint)) - v_0/obj.a_max(joint) + obj.a_max(joint)*(((v_0 + (a_0*(a_0 - obj.a_max(joint)))/(2*obj.j_max(joint)))/obj.a_max(joint) - obj.a_max(joint)/(2*obj.j_max(joint)) + (a_0 - obj.a_max(joint))/(2*obj.j_max(joint)))/obj.a_max(joint) + 1/obj.j_max(joint)) - (a_0^2 + 2*a_0*obj.a_max(joint) + 4*obj.a_max(joint)^2 - 2*obj.j_max(joint)*t_required*obj.a_max(joint) + 2*obj.j_max(joint)*v_0)/(2*obj.a_max(joint)*obj.j_max(joint)) + (a_0 + obj.a_max(joint))^2/(2*obj.a_max(joint)*obj.j_max(joint)) - (a_0*(a_0 + obj.a_max(joint)))/(obj.a_max(joint)*obj.j_max(joint)));
            
            % Check if v_drive is real and positive
            if ~imag(v_drive) && v_drive > 0
                [t, ~, mod_jerk_profile] = optSwitchTimes(obj, q_goal, q_0, dir*v_0, dir*a_0, joint, v_drive);

                % Check time constraint was fulfilled
                if t_required - t(end) < tol && t_required - t(end) > -tol/10
                    return;
                end
            end

            % Standard jerk profile: Phase 2 does not exist
            root=roots([3, 12*obj.a_max(joint), (-24*obj.a_max(joint)*obj.j_max(joint)*t_required - 12*a_0^2 - 24*a_0*obj.a_max(joint) + 12*obj.a_max(joint)^2 + 24*obj.j_max(joint)*v_0), 0, 48*a_0^2*obj.a_max(joint)*obj.j_max(joint)*t_required - 96*dir*obj.j_max(joint)^2*obj.a_max(joint)*q_0 + 96*dir*obj.j_max(joint)^2*obj.a_max(joint)*q_goal - 96*obj.a_max(joint)*obj.j_max(joint)^2*v_0*t_required + 12*a_0^4 + 16*a_0^3*obj.a_max(joint) - 24*a_0^2*obj.a_max(joint)^2 - 48*a_0^2*obj.j_max(joint)*v_0 + 48*obj.a_max(joint)^2*obj.j_max(joint)*v_0 + 48*obj.j_max(joint)^2*v_0^2]);
            v_drive = (-2*a_0^2 + 4*obj.j_max(joint)*v_0 + root(3)^2)/(4*obj.j_max(joint));
            
            % Check if v_drive is real and positive
            if ~imag(v_drive) && v_drive > 0
                [t, ~, mod_jerk_profile] = optSwitchTimes(obj, q_goal, q_0, dir*v_0, dir*a_0, joint, v_drive);

                % Check time constraint was fulfilled
                if t_required - t(end) < tol && t_required - t(end) > -tol/10
                    return;
                end
            end

            % Standard jerk profile: Phase 6 does not exist
            root = roots([12, 24*obj.a_max(joint), (-24*obj.a_max(joint)*obj.j_max(joint)*t_required + 24*a_0^2 - 48*a_0*obj.a_max(joint) + 24*obj.a_max(joint)^2 - 24*obj.j_max(joint)*v_0 + 12*a_0 - 12*obj.a_max(joint)), 0, -24*dir*obj.j_max(joint)^2*obj.a_max(joint)*q_0 + 24*dir*obj.j_max(joint)^2*obj.a_max(joint)*q_goal + 9*a_0^4 - 12*a_0^3*obj.a_max(joint) - 24*a_0^2*obj.j_max(joint)*v_0 + 48*a_0*obj.a_max(joint)*obj.j_max(joint)*v_0 + 4*obj.a_max(joint)^4 - 24*obj.a_max(joint)^2*obj.j_max(joint)*v_0 + 12*obj.j_max(joint)^2*v_0^2 + 6*a_0^3 + 6*a_0^2*obj.a_max(joint) - 12*a_0*obj.a_max(joint)^2 - 12*a_0*obj.j_max(joint)*v_0 + 12*obj.a_max(joint)*obj.j_max(joint)*v_0 + 4*a_0*obj.a_max(joint) - 4*obj.a_max(joint)^2]);
            v_drive = root(3)^2/obj.j_max(joint);
            
            % Check if v_drive is real and positive
            if ~imag(v_drive) && v_drive > 0
                [t, ~, mod_jerk_profile] = optSwitchTimes(obj, q_goal, q_0, dir*v_0, dir*a_0, joint, v_drive);

                % Check time constraint was fulfilled
                if t_required - t(end) < tol && t_required - t(end) > -tol/10
                    return;
                end
            end

            % Standard jerk profile: Phases 2 and 6 do not exist
            root = roots([(144*obj.j_max(joint)*t_required + 144*a_0), (-72*obj.j_max(joint)^2*t_required^2 - 144*a_0*obj.j_max(joint)*t_required + 36*a_0^2 - 216*obj.j_max(joint)*v_0), (144*dir*obj.j_max(joint)^2*q_0 - 144*dir*obj.j_max(joint)^2*q_goal + 48*a_0^3 - 144*a_0*obj.j_max(joint)*v_0), (-144*dir*obj.j_max(joint)^3*q_0*t_required + 144*dir*obj.j_max(joint)^3*q_goal*t_required - 48*a_0^3*obj.j_max(joint)*t_required - 144*a_0*dir*obj.j_max(joint)^2*q_0 + 144*a_0*dir*obj.j_max(joint)^2*q_goal + 144*a_0*obj.j_max(joint)^2*v_0*t_required + 6*a_0^4 - 72*a_0^2*obj.j_max(joint)*v_0 + 216*obj.j_max(joint)^2*v_0^2), 0, -72*dir^2*obj.j_max(joint)^4*q_0^2 + 144*dir^2*obj.j_max(joint)^4*q_0*q_goal - 72*dir^2*obj.j_max(joint)^4*q_goal^2 - 48*a_0^3*dir*obj.j_max(joint)^2*q_0 + 48*a_0^3*dir*obj.j_max(joint)^2*q_goal + 144*a_0*dir*obj.j_max(joint)^3*q_0*v_0 - 144*a_0*dir*obj.j_max(joint)^3*q_goal*v_0 + a_0^6 - 6*a_0^4*obj.j_max(joint)*v_0 + 36*a_0^2*obj.j_max(joint)^2*v_0^2 - 72*obj.j_max(joint)^3*v_0^3]);
            v_drive = root(2)^2/obj.j_max(joint);
            
            % Check if v_drive is real and positive
            if ~imag(v_drive) && v_drive > 0
                [t, ~, mod_jerk_profile] = optSwitchTimes(obj, q_goal, q_0, dir*v_0, dir*a_0, joint, v_drive);

                % Check time constraint was fulfilled
                if t_required - t(end) < tol && t_required - t(end) > -tol/10
                    return;
                end
            end

            % Modified profile: Phase 2 does not exist
            root = roots([3, - 6*sqrt(2)*obj.a_max(joint), (12*obj.a_max(joint)*obj.j_max(joint)*t_required - 6*a_0^2 - 12*a_0*obj.a_max(joint) - 6*obj.a_max(joint)^2 - 12*obj.j_max(joint)*v_0), 0, -12*a_0^2*obj.a_max(joint)*obj.j_max(joint)*t_required - 24*dir*obj.j_max(joint)^2*obj.a_max(joint)*q_0 + 24*dir*obj.j_max(joint)^2*obj.a_max(joint)*q_goal - 24*obj.a_max(joint)*obj.j_max(joint)^2*v_0*t_required + 3*a_0^4 + 4*a_0^3*obj.a_max(joint) + 6*a_0^2*obj.a_max(joint)^2 + 12*a_0^2*obj.j_max(joint)*v_0 + 12*obj.a_max(joint)^2*obj.j_max(joint)*v_0 + 12*obj.j_max(joint)^2*v_0^2]);
            v_drive = -(root(3)^2 - a_0^2 - 2*obj.j_max(joint)*v_0)/(2*obj.j_max(joint));
            
            % Check if v_drive is real and positive
            if ~imag(v_drive) && v_drive > 0
                [t, ~, mod_jerk_profile] = optSwitchTimes(obj, q_goal, q_0, dir*v_0, dir*a_0, joint, v_drive);

                % Check time constraint was fulfilled
                if t_required - t(end) < tol && t_required - t(end) > -tol/10
                    return;
                end
            end

            % Modified profile: Phase 6 does not exist
            root = roots([12, - 24*obj.a_max(joint), (24*obj.a_max(joint)*obj.j_max(joint)*t_required - 12*a_0^2 - 24*a_0*obj.a_max(joint) - 12*obj.a_max(joint)^2 - 24*obj.j_max(joint)*v_0), 0, 24*dir*obj.j_max(joint)^2*obj.a_max(joint)*q_0 - 24*dir*obj.j_max(joint)^2*obj.a_max(joint)*q_goal + 3*a_0^4 + 8*a_0^3*obj.a_max(joint) + 6*a_0^2*obj.a_max(joint)^2 + 12*a_0^2*obj.j_max(joint)*v_0 + 24*a_0*obj.a_max(joint)*obj.j_max(joint)*v_0 + 12*obj.a_max(joint)^2*obj.j_max(joint)*v_0 + 12*obj.j_max(joint)^2*v_0^2]);
            v_drive = root(3)^2/obj.j_max(joint);
            
            % Check if v_drive is real and positive
            if ~imag(v_drive) && v_drive > 0
                [t, ~, mod_jerk_profile] = optSwitchTimes(obj, q_goal, q_0, dir*v_0, dir*a_0, joint, v_drive);

                % Check time constraint was fulfilled
                if t_required - t(end) < tol && t_required - t(end) > -tol/10
                    return;
                end
            end

            % Modified profile: Phases 2 and 6 do not exist
            root = roots([144, (-144*obj.j_max(joint)*t_required + 144*a_0), (72*obj.j_max(joint)^2*t_required^2 - 144*a_0*obj.j_max(joint)*t_required - 36*a_0^2 - 216*obj.j_max(joint)*v_0), (-144*dir*obj.j_max(joint)^2*q_0 + 144*dir*obj.j_max(joint)^2*q_goal - 48*a_0^3 - 144*a_0*obj.j_max(joint)*v_0), (144*dir*obj.j_max(joint)^3*q_0*t_required - 144*dir*obj.j_max(joint)^3*q_goal*t_required + 48*a_0^3*obj.j_max(joint)*t_required - 144*a_0*dir*obj.j_max(joint)^2*q_0 + 144*a_0*dir*obj.j_max(joint)^2*q_goal + 144*a_0*obj.j_max(joint)^2*v_0*t_required + 6*a_0^4 + 72*a_0^2*obj.j_max(joint)*v_0 + 216*obj.j_max(joint)^2*v_0^2), 0, 72*dir^2*obj.j_max(joint)^4*q_0^2 - 144*dir^2*obj.j_max(joint)^4*q_0*q_goal + 72*dir^2*obj.j_max(joint)^4*q_goal^2 + 48*a_0^3*dir*obj.j_max(joint)^2*q_0 - 48*a_0^3*dir*obj.j_max(joint)^2*q_goal + 144*a_0*dir*obj.j_max(joint)^3*q_0*v_0 - 144*a_0*dir*obj.j_max(joint)^3*q_goal*v_0 - a_0^6 - 6*a_0^4*obj.j_max(joint)*v_0 - 36*a_0^2*obj.j_max(joint)^2*v_0^2 - 72*obj.j_max(joint)^3*v_0^3]);
            v_drive = root(4)^2/obj.j_max(joint);
            
            % Check if v_drive is real and positive
            if ~imag(v_drive) && v_drive > 0
                [t, ~, mod_jerk_profile] = optSwitchTimes(obj, q_goal, q_0, dir*v_0, dir*a_0, joint, v_drive);

                % Check time constraint was fulfilled
                if t_required - t(end) < tol && t_required - t(end) > -tol/10
                    return;
                end
            end

            % No valid solution found, reset return parameters
            mod_jerk_profile = false;
            t = zeros(1,7);
            v_drive = obj.v_max(joint);
        end
        
        function [q, t_rel, dir] = optBraking(obj, v_0, a_0, joint)
            % OPTBRAKING % Calculate time and joint angles required to
            % bring velocity to zero
            
            % This function can be used to:
            % - bring a joint to a full stop as fast as possible
            % - calculate in which direction a joint has to be actuated to
            %   reach a goal
            % - slow a joint down to a desired velocity (v_0 must be set to
            %   v_current - v_desired)
            
            % Set direction opposite to maximal velocity to be reached
            if v_0*a_0 > 0
                % v and a in same direction
                dir = -sign(v_0);
            else
                % If initial acceleration will cause the robot to
                % eventually move into opposite direction of velocity, use 
                % this direction
                if abs(v_0) > 1/2*a_0^2/obj.j_max(joint)
                    dir = -sign(v_0);
                else
                    dir = -sign(a_0);
                end
            end

            % If stopping dir. is negative, map scenario to pos. direction
            if(dir < 0)
                a_0 = -a_0;
                v_0 = -v_0;
            end

            % Bring velocity to zero
            t_rel(1) = (obj.a_max(joint) - a_0)/obj.j_max(joint);
            t_rel(3) = obj.a_max(joint)/obj.j_max(joint);
            t_rel(2) = (- v_0 - 1/2*t_rel(1)*a_0)/obj.a_max(joint) - 1/2*(t_rel(1) + t_rel(3));
            
            % Check if phase 2 does not exist 
            % (max acceleration is not reached)
            if(t_rel(2) < -obj.Tsample)
                t_rel(1) = -a_0/obj.j_max(joint) + sqrt(a_0^2/(2*obj.j_max(joint)^2) - v_0/obj.j_max(joint));
                t_rel(3) = t_rel(1) + a_0/obj.j_max(joint);
                t_rel(2) = 0;
            end
            
            % Calculate position after braking
            q = v_0*(t_rel(1) + t_rel(2) + t_rel(3)) + a_0*(1/2*t_rel(1)^2 + t_rel(1)*(t_rel(2) + t_rel(3)) + 1/2*t_rel(3)^2) + obj.j_max(joint)*(1/6*t_rel(1)^3 + 1/2*t_rel(1)^2*(t_rel(2) + t_rel(3)) - 1/6*t_rel(3)^3 + 1/2*t_rel(1)*t_rel(3)^2) + obj.a_max(joint)*(1/2*t_rel(2)^2 + t_rel(2)*t_rel(3));

            % Correct direction
            q = dir * q;
        end
        
        function [q_traj, v_traj, a_traj] = getTrajectories(varargin)
            %GETTRAJECTORIES % Calculate trajectory based on jerk switch
            % times
            
            % Apply transfer parameters
            obj = varargin{1};
            t = varargin{2};
            dir = varargin{3};
            mod_jerk_profile = varargin{4};
            q_0 = varargin{5};
            v_0 = varargin{6};
            a_0  = varargin{7};
            
            if nargin > 7
                v_drive = varargin{8};
            else
                v_drive = obj.v_max;
            end
            
            % Calculate length of trajectory in samples
            traj_len = max(ceil(t(:,7)/obj.Tsample)) + 1;
            
            % Jerk switch times in samples
            sampled_t = zeros(obj.DoF,7);

            %% Calculate jerk trajectories
            j_traj = zeros(obj.DoF,traj_len); 
            const_v = false(obj.DoF,1);
            for joint=1:obj.DoF

                % Save fractions lost when discretizing switch times
                sampled_t_trans = zeros(1,7); 
                
                % Check which jerk profile to use
                if mod_jerk_profile(joint)
                    % Only used for specific scenarios in time scaling
                    jerk_profile = dir(joint) * obj.j_max(joint) * [-1 0 1 0 -1 0 1];
                else
                    % Standard case
                    jerk_profile = dir(joint) * obj.j_max(joint) * [1 0 -1 0 -1 0 1];
                end
                
                % Calculate inaccuraties when sampling times
                for j = 1:7
                    sampled_t_trans(j) = mod(t(joint,j),obj.Tsample);
                end             

                % Round towards phases with zero jerk
                sampled_t(joint,1) = floor(t(joint,1)/obj.Tsample);
                sampled_t(joint,2) = ceil(t(joint,2)/obj.Tsample);
                sampled_t(joint,3) = floor(t(joint,3)/obj.Tsample);
                sampled_t(joint,4) = ceil(t(joint,4)/obj.Tsample);
                sampled_t(joint,5) = floor(t(joint,5)/obj.Tsample);
                sampled_t(joint,6) = ceil(t(joint,6)/obj.Tsample);
                sampled_t(joint,7) = floor(t(joint,7)/obj.Tsample);
                
                % Calculate sampled jerk trajectory
                if(sampled_t(joint,1) > 0)
                    j_traj(joint,1:sampled_t(joint,1)) = jerk_profile(1);
                end
                for j = 2:7
                    if(sampled_t(joint,j) - sampled_t(joint,j-1) > 0)
                        j_traj(joint,sampled_t(joint,j-1)+1:sampled_t(joint,j)) = jerk_profile(j);
                    end
                end
                
                %% Add partial jerk of sample fractions to increase accuracy
                if(sampled_t(joint,3) >= sampled_t(joint,2))
                    % Phase 2 exists: 
                    % Fractions can be added to its beginning and end
                    j_traj(joint,sampled_t(joint,1) + 1) = j_traj(joint,sampled_t(joint,1) + 1) + sampled_t_trans(1)/obj.Tsample * jerk_profile(1);
                    if(sampled_t(joint,2) > 0)
                        j_traj(joint,sampled_t(joint,2)) = j_traj(joint,sampled_t(joint,2)) + (1 - sampled_t_trans(2)/obj.Tsample) * jerk_profile(3);
                    end
                    % Add fraction to end of phase 3
                    j_traj(joint,sampled_t(joint,3) + 1) = j_traj(joint,sampled_t(joint,3) + 1) + sampled_t_trans(3)/obj.Tsample * jerk_profile(3);
                else
                    % Phase 2 does not exist: 
                    % Calculate transition sample
                    if(sampled_t(joint,2) > 0)
                        j_traj(joint,sampled_t(joint,2)) = j_traj(joint,sampled_t(joint,2)) + sampled_t_trans(1)/obj.Tsample * jerk_profile(1) + (sampled_t_trans(3)-sampled_t_trans(1))/obj.Tsample * jerk_profile(3);
                    end
                end
                
                % Add fraction to end of phase 4
                if(sampled_t(joint,4) > 0)
                    j_traj(joint,sampled_t(joint,4)) = j_traj(joint,sampled_t(joint,4)) + (1 - sampled_t_trans(4)/obj.Tsample) * jerk_profile(5);
                end
                
                if(sampled_t(joint,3) - sampled_t(joint,1) > 0)
                    % Phase 2 and/ or 3 exist:
                    % Add fraction to beginning of phase 6
                    j_traj(joint,sampled_t(joint,5) + 1) = j_traj(joint,sampled_t(joint,5) + 1) + sampled_t_trans(5)/obj.Tsample * jerk_profile(5);
                else
                    % Phase 2 and 3 do not exist:
                    % Add fractions from previous phases to end of phase 5
                    if(sampled_t(joint,5) > 0)
                        j_traj(joint,sampled_t(joint,5)) = j_traj(joint,sampled_t(joint,5)) + sampled_t_trans(5)/obj.Tsample * jerk_profile(5)  + sampled_t_trans(1)/obj.Tsample * jerk_profile(1) + (sampled_t_trans(3)-sampled_t_trans(1))/obj.Tsample * jerk_profile(3);
                    end
                end
                
                % Add fraction to end of phase 4
                if(sampled_t(joint,6) > 0)
                    j_traj(joint,sampled_t(joint,6)) = j_traj(joint,sampled_t(joint,6)) + (1 - sampled_t_trans(6)/obj.Tsample) * jerk_profile(7);
                end
                
                % Add fraction to end of trajectory (after phase 7)
                j_traj(joint,sampled_t(joint,7) + 1) = j_traj(joint,sampled_t(joint,7) + 1) + sampled_t_trans(7)/obj.Tsample * jerk_profile(7);

                % Check if phase 4 exists (constant velocity)
                if sampled_t(joint,4) - sampled_t(joint,3) > 2
                    const_v(joint) = true;
                end
            end

            %% Calculate acceleration trajectories
            a_traj = obj.Tsample * cumsum(j_traj,2) + a_0;

            % Set final acceleration to exactly 0 (increases accuracy)
            a_traj(joint,sampled_t(joint,7)+1:end) = 0;

            %% Calculate velocitie trajectories
            v_traj = obj.Tsample * cumsum(a_traj,2) + v_0;

            % Set constant velocity periods to exactly v_drive
            % (increases accuracy)
            for joint=1:obj.DoF
                if const_v(joint)
                    v_traj(joint,sampled_t(joint,3)+1:sampled_t(joint,4)-1) = v_drive(joint) * dir(joint);
                end

                % Set final velocity to exactly 0 (increases accuracy)
                v_traj(joint,sampled_t(joint,7)+1:end) = 0;
            end

            %% Calculate joint angle trajectories
            q_traj = obj.Tsample * cumsum(v_traj,2) + q_0;
        end
    end
end
