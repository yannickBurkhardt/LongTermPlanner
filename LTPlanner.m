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
            %SETLIMITS Change maximal jerk, acceleration and velocity for
            %every joint
            obj.j_max = j_max;
            obj.a_max = a_max;
            obj.v_max = v_max;
        end
        
        function checkInputs(obj, v_0, a_0)
            for i=1:obj.DoF
                % Check input velocity and acceleration
                if(abs(v_0(i)) > obj.v_max(i))
                    error("Input velocity exceeds limits.")
                end
                if(abs(a_0(i)) > obj.a_max(i))
                    error("Input acceleration exceeds limits.")
                end
                if (abs(v_0(i) + 1/2*a_0(i)*abs(a_0(i))/obj.j_max(i)) > obj.v_max(i))
                    error("Input velocity will exceed limits because acceleration is too high.")
                end
            end
        end
        
        function [t, mod_jerk_profile] = optSwitchTimes(varargin)
            %OPTSWITCHTIMES Calculate time-optimal jerk swtiches
            
            obj = varargin{1};
            q_goal = varargin{2};
            q_0 = varargin{3};
            v_0 = varargin{4};
            a_0 = varargin{5};
            
            if nargin>5
                v_drive = varargin{6};
            else
                v_drive = obj.v_max;
            end
            
            % Parameters for calculation
            dir = zeros(obj.DoF,1); % Direction of the goal (only +1 or -1)
            t_rel = zeros(obj.DoF,7); % Time that is required for one jerk phase
            t = zeros(obj.DoF,7); % Absolute time that is required to reach the end of current jerk phase
            mod_jerk_profile = false; % Use the standard jerk profile if not changed during calculations
            eps = 1e-4;
            
            % Check if inputs are in limits
            checkInputs(obj, v_0, a_0);
            
            for i=1:obj.DoF
                
                % Calculate direction of movement
                [q_stop, t_rel(i, 1:3)] = getStopPos(obj, v_0, a_0, i);
                q_diff = q_goal(i) - (q_0(i) + q_stop);
                if (abs(q_diff) < eps)
                    % Skip rest if that fulfils scenario
                    continue;
                end
                dir(i) = sign(q_diff);

                % If goal is in negative direction, map to pos. direction
                if(dir(i) < 0)
                    v_0(i) = -v_0(i);
                    a_0(i) = -a_0(i);
                end

                %% Check if slowing down is necessary to satisfy v_max
                if(v_0(i) + 1/2*a_0(i)*abs(a_0(i))/obj.j_max(i) > v_drive)
                    mod_jerk_profile = true;
                end

                q_break = 0;
                if mod_jerk_profile
                    [q_break, t_rel(i,1:3)] = getStopPos(obj, v_0(i) - v_drive, a_0, i);
                else
                    
                    %% Calculate min. time required per joint to reach goal state

                    % j = +- j_max (const) -> linear a
                    t_rel(i,1) = (obj.a_max(i) - a_0(i))/obj.j_max(i);
                    t_rel(i,3) = obj.a_max(i)/obj.j_max(i);
                    
                    % a = +- a_max (const) -> linear v
                    % works but only if all times > 0
                    t_rel(i,2) = (v_drive - v_0(i) - 1/2*t_rel(i,1)*a_0(i))/obj.a_max(i) - 1/2*(t_rel(i,1) + t_rel(i,3));

                    % Check if max acceleration cannot be reached
                    if(t_rel(i,2) < -eps)
                        % Check if root is positive
                        root = obj.j_max(i)*(v_drive - v_0(i)) + 1/2*a_0(i)^2;
                        if(root > 0)
                            t_rel(i,3) = sqrt(root)/obj.j_max(i);
                            t_rel(i,1) = t_rel(i,3) - a_0(i)/obj.j_max(i);
                            t_rel(i,2) = 0;
                        else
                            error("Negative root in t_rel(" + i + ",2): " + root)
                        end
                    end
                end

                % j = +- j_max (const) -> linear a
                t_rel(i,5) = obj.a_max(i)/obj.j_max(i);
                t_rel(i,7) = t_rel(i,5);

                % a = +- a_max (const) -> linear v
                % works but only if all times > 0
                t_rel(i,6) = v_drive/obj.a_max(i) - 1/2*(t_rel(i,5) + t_rel(i,7));

                % Check if max acceleration cannot be reached
                if(t_rel(i,6) < -eps)
                    % Check if root is positive
                    root = v_drive/obj.j_max(i);
                    if(root > 0)
                        t_rel(i,5) = sqrt(root);
                        t_rel(i,7) = t_rel(i,5);
                        t_rel(i,6) = 0;
                    else
                        %error("Negative root in t_rel(" + i + ",6): " + root)
                        t = zeros(obj.DoF,7);
                        continue;
                    end
                end

                % v = v_max (const) -> linear q
                if(mod_jerk_profile)
                    q_part1 = q_break + v_drive(i)*(t_rel(i,1) + t_rel(i,2) + t_rel(i,3));
                else
                    q_part1 = v_0(i)*(t_rel(i,1) + t_rel(i,2) + t_rel(i,3)) + a_0(i)*(1/2*t_rel(i,1)^2 + t_rel(i,1)*(t_rel(i,2) + t_rel(i,3)) + 1/2*t_rel(i,3)^2) + obj.j_max(i)*(1/6*t_rel(i,1)^3 + 1/2*t_rel(i,1)^2*(t_rel(i,2) + t_rel(i,3)) - 1/6*t_rel(i,3)^3 + 1/2*t_rel(i,1)*t_rel(i,3)^2) + obj.a_max(i)*(1/2*t_rel(i,2)^2 + t_rel(i,2)*t_rel(i,3));
                end
                q_part2 = obj.j_max(i)*(1/6*t_rel(i,7)^3 + 1/2*t_rel(i,7)^2*(t_rel(i,6) + t_rel(i,5)) - 1/6*t_rel(i,5)^3 + 1/2*t_rel(i,7)*t_rel(i,5)^2) + obj.a_max(i)*(1/2*t_rel(i,6)^2 + t_rel(i,6)*t_rel(i,5));
                t_rel(i,4) = ((q_goal(i) - q_0(i))*dir - q_part1 - q_part2)/v_drive;

                % Check if max velocity cannot be reached
                if(t_rel(i,4) < -eps)
                    
                    if(mod_jerk_profile)
                        % This case is not valid
                        %error("Input velocity exceeds limits.")
                        t_rel = zeros(obj.DoF,7);
                        continue;
                    end
                    
                    % Formula calculated with matlab
                    % Check if root is positive
                    root = (obj.j_max(i)^2*t_rel(i,1)^4)/2 - (obj.j_max(i)^2*t_rel(i,3)^4)/4 + (obj.j_max(i)^2*t_rel(i,3)^2*t_rel(i,5)^2)/2 - (obj.j_max(i)^2*t_rel(i,5)^4)/4 + (obj.j_max(i)^2*t_rel(i,7)^4)/2 + 2*obj.j_max(i)*a_0(i)*t_rel(i,1)^3 - (2*obj.j_max(i)*obj.a_max(i)*t_rel(i,1)^3)/3 - 2*obj.j_max(i)*obj.a_max(i)*t_rel(i,1)*t_rel(i,3)^2 + (2*obj.j_max(i)*obj.a_max(i)*t_rel(i,3)^3)/3 + (2*obj.j_max(i)*obj.a_max(i)*t_rel(i,5)^3)/3 - 2*obj.j_max(i)*obj.a_max(i)*t_rel(i,5)^2*t_rel(i,7) - (2*obj.j_max(i)*obj.a_max(i)*t_rel(i,7)^3)/3 + 2*obj.j_max(i)*v_0(i)*t_rel(i,1)^2 + 2*a_0(i)^2*t_rel(i,1)^2 - 2*a_0(i)*obj.a_max(i)*t_rel(i,1)^2 - 2*a_0(i)*obj.a_max(i)*t_rel(i,3)^2 + 4*a_0(i)*v_0(i)*t_rel(i,1) + 2*obj.a_max(i)^2*t_rel(i,3)^2 + 2*obj.a_max(i)^2*t_rel(i,5)^2 - 4*obj.a_max(i)*v_0(i)*t_rel(i,1) + 4*dir*(q_goal(i) - q_0(i))*obj.a_max(i) + 2*v_0(i)^2;
                    if(0 < root)
                        t_rel(i,6) = -(4*obj.a_max(i)*t_rel(i,5) - 2*root^(1/2) + obj.j_max(i)*t_rel(i,3)^2 - obj.j_max(i)*t_rel(i,5)^2 + 2*obj.j_max(i)*t_rel(i,7)^2)/(4*obj.a_max(i));
                        t_rel(i,2) = (-v_0(i) - a_0(i)*t_rel(i,1) - 1/2*obj.j_max(i)*t_rel(i,1)^2 + 1/2*obj.j_max(i)*t_rel(i,3)^2 + 1/2*obj.j_max(i)*t_rel(i,7)^2 - 1/2*obj.j_max(i)*t_rel(i,5)^2)/obj.a_max(i) - t_rel(i,3) + t_rel(i,6) + t_rel(i,5);
                        t_rel(i,4) = 0;
                    else
                        %error("Negative root in t_rel(" + i + ",4): " + root)
                        t = zeros(obj.DoF,7);
                        continue;
                    end

                    % Check if max velocity and max acceleration cannot be reached
                    if(t_rel(i,6) < -eps || t_rel(i,2) < -eps)
                        root = roots([12, 0, (-24*a_0(i)^2 + 48*obj.j_max(i)*v_0(i)), (48*dir(i)*obj.j_max(i)^2*q_0(i) - 48*dir(i)*obj.j_max(i)^2*q_goal(i) + 16*a_0(i)^3 - 48*a_0(i)*obj.j_max(i)*v_0(i)), -3*a_0(i)^4 + 12*a_0(i)^2*obj.j_max(i)*v_0(i) - 12*obj.j_max(i)^2*v_0(i)^2]);
                        
                        % Choose non-complex, positive solution
                        root = root((abs(imag(root)) < eps));
                        root = root(root >= 0);
                        t_rel(i,1) = (2*root(1)^2 - 4*a_0(i)*root(1) + a_0(i)^2 - 2*v_0(i)*obj.j_max(i))/(4*obj.j_max(i)*root(1));
                        t_rel = real(t_rel);
                        
                        % Calculate other switch times
                        t_rel(i,7) = sqrt(4*obj.j_max(i)^2*t_rel(i,1)^2 + 8*a_0(i)*obj.j_max(i)*t_rel(i,1) + 2*a_0(i)^2 + 4*obj.j_max(i)*v_0(i))/(2*obj.j_max(i));
                        t_rel(i,5) = a_0(i)/obj.j_max(i) + t_rel(i,1) + t_rel(i,7);
                        t_rel(i,2) = 0;
                        t_rel(i,6) = 0;

                        % Check if a_max is exceeded
                        if(a_0(i) + t_rel(i,1)*obj.j_max(i) > obj.a_max(i))
                            t_rel(i,1) = (obj.a_max(i) - a_0(i)) / obj.j_max(i);
                            t_rel(i,7) = 1/obj.j_max(i) * (obj.a_max(i)/2 + sqrt(9*obj.a_max(i)^2 + 6*sqrt(-12*obj.a_max(i)*obj.j_max(i)^3*t_rel(i,1)^3 + 9*a_0(i)^2*obj.j_max(i)^2*t_rel(i,1)^2 - 18*a_0(i)*obj.a_max(i)*obj.j_max(i)^2*t_rel(i,1)^2 + 9*obj.a_max(i)^2*obj.j_max(i)^2*t_rel(i,1)^2 + 36*a_0(i)*obj.j_max(i)^2*t_rel(i,1)*v_0(i) - 72*obj.a_max(i)*dir(i)*obj.j_max(i)^2*q_0(i) + 72*obj.a_max(i)*dir(i)*obj.j_max(i)^2*q_goal(i) - 36*obj.a_max(i)*obj.j_max(i)^2*t_rel(i,1)*v_0(i) + 3*obj.a_max(i)^4 + 36*obj.j_max(i)^2*v_0(i)^2))/6 - obj.a_max(i));
                            t_rel(i,5) = t_rel(i,7) + obj.a_max(i)/obj.j_max(i);
                            t_rel(i,2) = -(-obj.j_max(i)*t_rel(i,5)^2 - 2*obj.j_max(i)*t_rel(i,5)*t_rel(i,7) + obj.j_max(i)*t_rel(i,7)^2 + a_0(i)*t_rel(i,1) + obj.a_max(i)*t_rel(i,1) + 2*obj.a_max(i)*t_rel(i,5) + 2*obj.a_max(i)*t_rel(i,7) + 2*v_0(i))/(2*obj.a_max(i));
                            t_rel(i,6) = 0;

                        % Check if -a_max is exceeded
                        elseif(t_rel(i,7)*obj.j_max(i) > obj.a_max(i))
                            t_rel(i,7) = obj.a_max(i)/obj.j_max(i);
                            root = roots([12, - 24*obj.a_max(i), (-12*a_0(i)^2 + 12*obj.a_max(i)^2 + 24*obj.j_max(i)*v_0(i)), 0, 24*dir(i)*obj.j_max(i)^2*q_0(i)*obj.a_max(i) - 24*dir(i)*obj.j_max(i)^2*q_goal(i)*obj.a_max(i) + 3*a_0(i)^4 + 8*a_0(i)^3*obj.a_max(i) + 6*a_0(i)^2*obj.a_max(i)^2 - 12*a_0(i)^2*obj.j_max(i)*v_0(i) - 24*a_0(i)*obj.j_max(i)*v_0(i)*obj.a_max(i) - 12*obj.a_max(i)^2*obj.j_max(i)*v_0(i) + 12*obj.j_max(i)^2*v_0(i)^2]);
                            
                            % Choose non-complex, positive solution
                            root = root((abs(imag(root)) < eps));
                            root = root(root >= 0);
                            t_rel(i,1) = (root - a_0(i) - obj.a_max(i))/obj.j_max(i);
                            
                            % Calculate other switch times
                            t_rel(i,5) = (a_0(i) + obj.a_max(i))/obj.j_max(i) + t_rel(i,1);
                            t_rel(i,6) = (obj.j_max^2*t_rel(i,1)^2 + 2*obj.j_max(i)^2*t_rel(i,1)*t_rel(i,5) - obj.j_max(i)^2*t_rel(i,5)^2 + 2*a_0(i)*obj.j_max(i)*t_rel(i,1) + 2*a_0(i)*obj.j_max(i)*t_rel(i,5) - obj.a_max(i)^2 + 2*obj.j_max(i)*v_0(i))/(2*obj.j_max(i)*obj.a_max(i));
                            t_rel(i,2) = 0;
                        end

                        % All other times are 0
                        t_rel(i,3) = 0;
                        t_rel(i,4) = 0;
                    end
                end
            end

            % Safety checks
            for j=1:7
                if(t_rel(i,j) < 0)
                    if(t_rel(i,j) < -eps)
                        % No numeric inaccuracy
                        t = zeros(obj.DoF,7);
                        %error("t_rel(" + i + "," + j + ") is negative: " + t_rel(i,j))
                    end
                    t_rel(i,j) = 0;
                end
                if any(abs(imag(t_rel(i,j))) > 0)
                    if any(abs(imag(t_rel(i,j))) > eps)
                        % No numeric inaccuracy
                        t = zeros(obj.DoF,7);
                        %error("t_rel(" + i + "," + j + ") is complex: " + t_rel(i,j))
                    end
                    t_rel(i,j) = real(t_rel(i,j));
                end
            end

            % Calculate absolute times for jerk switches
            t(i,:) = cumsum(t_rel(i,:));
        end
        
        function [t, mod_jerk_profile] = timeScaling(obj, q_goal, q_0, v_0, a_0, t_required)
            % TIMESCALING % Calculate switching times to fulfil a given
            % time by adjusting the maximally reached velocity
            
            % Parameters for calculation
            dir = zeros(obj.DoF,1); % Direction of the goal (only +1 or -1)
            t_rel = zeros(obj.DoF,7); % Time that is required for one jerk phase
            t = zeros(obj.DoF,7); % Absolute time that is required to reach the end of current jerk phase
            mod_jerk_profile = false;
            eps = 1e-4;
            tol = 0.1;

            %% Analyse input data
            % Check if inputs are in limits
            % checkInputs(obj, v_0, a_0); %TODO
            
            for i=1:obj.DoF
                
                % Calculate direction of movement
                [q_stop, t_rel(i, 1:3)] = getStopPos(obj, v_0, a_0, i);
                q_diff = q_goal(i) - (q_0(i) + q_stop);
                if (abs(q_diff) < eps)
                    % Skip rest if that fulfils scenario
                    t(i,:) = cumsum(t_rel(i,:));
                    continue;
                end
                dir(i) = sign(q_diff);

                % If goal is in negative direction, map to pos. direction
                if(dir(i) < 0)
                    v_0(i) = -v_0(i);
                    a_0(i) = -a_0(i);
                end

                %% Calculate required v_drive to reach goal at given time
                % Standard jerk profile: Phases 2 and 6 exist
                v_drive = (obj.a_max(i)*obj.j_max(i)*t_required/2 - a_0(i)^2/4 + a_0(i)*obj.a_max(i)/2 - obj.a_max(i)^2/2 + v_0(i)*obj.j_max(i)/2 - sqrt(36*obj.a_max(i)^2*obj.j_max(i)^2*t_required^2 - 36*a_0(i)^2*obj.a_max(i)*obj.j_max(i)*t_required + 72*a_0(i)*obj.a_max(i)^2*obj.j_max(i)*t_required - 72*obj.a_max(i)^3*obj.j_max(i)*t_required + 144*obj.a_max(i)*dir(i)*obj.j_max(i)^2*q_0(i) - 144*obj.a_max(i)*dir(i)*obj.j_max(i)^2*q_goal(i) + 72*obj.a_max(i)*obj.j_max(i)^2*v_0(i)*t_required - 9*a_0(i)^4 + 12*a_0(i)^3*obj.a_max(i) + 36*a_0(i)^2*obj.a_max(i)^2 + 36*a_0(i)^2*obj.j_max(i)*v_0(i) - 72*a_0(i)*obj.a_max(i)^3 - 72*a_0(i)*obj.a_max(i)*obj.j_max(i)*v_0(i) + 36*obj.a_max(i)^4 - 36*obj.j_max(i)^2*v_0(i)^2)/12)/obj.j_max(i);
                
                if ~imag(v_drive) && v_drive > 0
                    [t, mod_jerk_profile] = optSwitchTimes(obj, q_goal, q_0, dir*v_0, dir*a_0, v_drive);
                    
                    % Check time constraint was fulfilled
                    if abs(t_required - t(end)) < tol
                        continue;
                    end
                end
                
                % Standard jerk profile: Phase 2 does not exist
                root=roots([3, 12*obj.a_max(i), (-24*obj.a_max(i)*obj.j_max(i)*t_required - 12*a_0(i)^2 - 24*a_0(i)*obj.a_max(i) + 12*obj.a_max(i)^2 + 24*obj.j_max(i)*v_0(i)), 0, 48*a_0(i)^2*obj.a_max(i)*obj.j_max(i)*t_required - 96*dir(i)*obj.j_max(i)^2*obj.a_max(i)*q_0(i) + 96*dir(i)*obj.j_max(i)^2*obj.a_max(i)*q_goal(i) - 96*obj.a_max(i)*obj.j_max(i)^2*v_0(i)*t_required + 12*a_0(i)^4 + 16*a_0(i)^3*obj.a_max(i) - 24*a_0(i)^2*obj.a_max(i)^2 - 48*a_0(i)^2*obj.j_max(i)*v_0(i) + 48*obj.a_max(i)^2*obj.j_max(i)*v_0(i) + 48*obj.j_max(i)^2*v_0(i)^2]);
                v_drive = (-2*a_0(i)^2 + 4*obj.j_max(i)*v_0(i) + root(3)^2)/(4*obj.j_max(i));
                if ~imag(v_drive) && v_drive > 0
                    [t, mod_jerk_profile] = optSwitchTimes(obj, q_goal, q_0, dir*v_0, dir*a_0, v_drive);
                    
                    % Check time constraint was fulfilled
                    if abs(t_required - t(end)) < tol
                        continue;
                    end
                end
                
                % Standard jerk profile: Phase 6 does not exist
                root = roots([12, 24*obj.a_max(i), (-24*obj.a_max(i)*obj.j_max(i)*t_required + 24*a_0(i)^2 - 48*a_0(i)*obj.a_max(i) + 24*obj.a_max(i)^2 - 24*obj.j_max(i)*v_0(i) + 12*a_0(i) - 12*obj.a_max(i)), 0, -24*dir(i)*obj.j_max(i)^2*obj.a_max(i)*q_0(i) + 24*dir(i)*obj.j_max(i)^2*obj.a_max(i)*q_goal(i) + 9*a_0(i)^4 - 12*a_0(i)^3*obj.a_max(i) - 24*a_0(i)^2*obj.j_max(i)*v_0(i) + 48*a_0(i)*obj.a_max(i)*obj.j_max(i)*v_0(i) + 4*obj.a_max(i)^4 - 24*obj.a_max(i)^2*obj.j_max(i)*v_0(i) + 12*obj.j_max(i)^2*v_0(i)^2 + 6*a_0(i)^3 + 6*a_0(i)^2*obj.a_max(i) - 12*a_0(i)*obj.a_max(i)^2 - 12*a_0(i)*obj.j_max(i)*v_0(i) + 12*obj.a_max(i)*obj.j_max(i)*v_0(i) + 4*a_0(i)*obj.a_max(i) - 4*obj.a_max(i)^2]);
                v_drive = root(3)^2/obj.j_max(i);
                if ~imag(v_drive) && v_drive > 0
                    [t, mod_jerk_profile] = optSwitchTimes(obj, q_goal, q_0, dir*v_0, dir*a_0, v_drive);
                    
                    % Check time constraint was fulfilled
                    if abs(t_required - t(end)) < tol
                        continue;
                    end
                end
                
                % Standard jerk profile: Phases 2 and 6 do not exist
                root = roots([(144*obj.j_max(i)*t_required + 144*a_0(i)), (-72*obj.j_max(i)^2*t_required^2 - 144*a_0(i)*obj.j_max(i)*t_required + 36*a_0(i)^2 - 216*obj.j_max(i)*v_0(i)), (144*dir(i)*obj.j_max(i)^2*q_0(i) - 144*dir(i)*obj.j_max(i)^2*q_goal(i) + 48*a_0(i)^3 - 144*a_0(i)*obj.j_max(i)*v_0(i)), (-144*dir(i)*obj.j_max(i)^3*q_0(i)*t_required + 144*dir(i)*obj.j_max(i)^3*q_goal(i)*t_required - 48*a_0(i)^3*obj.j_max(i)*t_required - 144*a_0(i)*dir(i)*obj.j_max(i)^2*q_0(i) + 144*a_0(i)*dir(i)*obj.j_max(i)^2*q_goal(i) + 144*a_0(i)*obj.j_max(i)^2*v_0(i)*t_required + 6*a_0(i)^4 - 72*a_0(i)^2*obj.j_max(i)*v_0(i) + 216*obj.j_max(i)^2*v_0(i)^2), 0, -72*dir(i)^2*obj.j_max(i)^4*q_0(i)^2 + 144*dir(i)^2*obj.j_max(i)^4*q_0(i)*q_goal(i) - 72*dir(i)^2*obj.j_max(i)^4*q_goal(i)^2 - 48*a_0(i)^3*dir(i)*obj.j_max(i)^2*q_0(i) + 48*a_0(i)^3*dir(i)*obj.j_max(i)^2*q_goal(i) + 144*a_0(i)*dir(i)*obj.j_max(i)^3*q_0(i)*v_0(i) - 144*a_0(i)*dir(i)*obj.j_max(i)^3*q_goal(i)*v_0(i) + a_0(i)^6 - 6*a_0(i)^4*obj.j_max(i)*v_0(i) + 36*a_0(i)^2*obj.j_max(i)^2*v_0(i)^2 - 72*obj.j_max(i)^3*v_0(i)^3]);
                v_drive = root(2)^2/obj.j_max(i);
                if ~imag(v_drive) && v_drive > 0
                    [t, mod_jerk_profile] = optSwitchTimes(obj, q_goal, q_0, dir*v_0, dir*a_0, v_drive);

                    % Check time constraint was fulfilled
                    if abs(t_required - t(end)) < tol
                        continue;
                    end
                end
                
                % Modified jerk profile: Phases 2 and 6 exist
                v_drive = -(dir(i)*(q_0(i) - q_goal(i)) - obj.j_max(i)*((a_0(i) + obj.a_max(i))^3/(6*obj.j_max(i)^3) - obj.a_max(i)^3/(6*obj.j_max(i)^3) + (obj.a_max(i)^2*(a_0(i) + obj.a_max(i)))/(2*obj.j_max(i)^3) + ((a_0(i) + obj.a_max(i))^2*((v_0(i) + (a_0(i)*(a_0(i) - obj.a_max(i)))/(2*obj.j_max(i)))/obj.a_max(i) + obj.a_max(i)/(2*obj.j_max(i)) + (a_0(i) - obj.a_max(i))/(2*obj.j_max(i))))/(2*obj.j_max(i)^2)) + a_0(i)*((a_0(i) + obj.a_max(i))^2/(2*obj.j_max(i)^2) + obj.a_max(i)^2/(2*obj.j_max(i)^2) + ((a_0(i) + obj.a_max(i))*((v_0(i) + (a_0(i)*(a_0(i) - obj.a_max(i)))/(2*obj.j_max(i)))/obj.a_max(i) + obj.a_max(i)/(2*obj.j_max(i)) + (a_0(i) - obj.a_max(i))/(2*obj.j_max(i))))/obj.j_max(i)) - obj.a_max(i)*(((v_0(i) + (a_0(i)*(a_0(i) - obj.a_max(i)))/(2*obj.j_max(i)))/obj.a_max(i) - obj.a_max(i)/(2*obj.j_max(i)) + (a_0(i) - obj.a_max(i))/(2*obj.j_max(i)))^2/2 + (obj.a_max(i)*((v_0(i) + (a_0(i)*(a_0(i) - obj.a_max(i)))/(2*obj.j_max(i)))/obj.a_max(i) - obj.a_max(i)/(2*obj.j_max(i)) + (a_0(i) - obj.a_max(i))/(2*obj.j_max(i))))/obj.j_max(i)) + v_0(i)*((v_0(i) + (a_0(i)*(a_0(i) - obj.a_max(i)))/(2*obj.j_max(i)))/obj.a_max(i) + (a_0(i) + obj.a_max(i))/obj.j_max(i) + obj.a_max(i)/(2*obj.j_max(i)) + (a_0(i) - obj.a_max(i))/(2*obj.j_max(i))))/(obj.a_max(i)/(2*obj.j_max(i)) - v_0(i)/obj.a_max(i) + obj.a_max(i)*(((v_0(i) + (a_0(i)*(a_0(i) - obj.a_max(i)))/(2*obj.j_max(i)))/obj.a_max(i) - obj.a_max(i)/(2*obj.j_max(i)) + (a_0(i) - obj.a_max(i))/(2*obj.j_max(i)))/obj.a_max(i) + 1/obj.j_max(i)) - (a_0(i)^2 + 2*a_0(i)*obj.a_max(i) + 4*obj.a_max(i)^2 - 2*obj.j_max(i)*t_required*obj.a_max(i) + 2*obj.j_max(i)*v_0(i))/(2*obj.a_max(i)*obj.j_max(i)) + (a_0(i) + obj.a_max(i))^2/(2*obj.a_max(i)*obj.j_max(i)) - (a_0(i)*(a_0(i) + obj.a_max(i)))/(obj.a_max(i)*obj.j_max(i)));
                if ~imag(v_drive) && v_drive > 0
                    [t, mod_jerk_profile] = optSwitchTimes(obj, q_goal, q_0, dir*v_0, dir*a_0, v_drive);

                    % Check time constraint was fulfilled
                    if abs(t_required - t(end)) < tol
                        continue;
                    end
                end
                
                % Modified profile: Phase 2 does not exist
                root = roots([3, - 6*sqrt(2)*obj.a_max(i), (12*obj.a_max(i)*obj.j_max(i)*t_required - 6*a_0(i)^2 - 12*a_0(i)*obj.a_max(i) - 6*obj.a_max(i)^2 - 12*obj.j_max(i)*v_0(i)), 0, -12*a_0(i)^2*obj.a_max(i)*obj.j_max(i)*t_required - 24*dir(i)*obj.j_max(i)^2*obj.a_max(i)*q_0(i) + 24*dir(i)*obj.j_max(i)^2*obj.a_max(i)*q_goal(i) - 24*obj.a_max(i)*obj.j_max(i)^2*v_0(i)*t_required + 3*a_0(i)^4 + 4*a_0(i)^3*obj.a_max(i) + 6*a_0(i)^2*obj.a_max(i)^2 + 12*a_0(i)^2*obj.j_max(i)*v_0(i) + 12*obj.a_max(i)^2*obj.j_max(i)*v_0(i) + 12*obj.j_max(i)^2*v_0(i)^2]);
                v_drive = -(root(3)^2 - a_0(i)^2 - 2*obj.j_max(i)*v_0(i))/(2*obj.j_max(i));
                if ~imag(v_drive) && v_drive > 0
                    [t, mod_jerk_profile] = optSwitchTimes(obj, q_goal, q_0, dir*v_0, dir*a_0, v_drive);

                    % Check time constraint was fulfilled
                    if abs(t_required - t(end)) < tol
                        continue;
                    end
                end
                
                % Modified profile: Phase 6 does not exist
                root = roots([12, - 24*obj.a_max(i), (24*obj.a_max(i)*obj.j_max(i)*t_required - 12*a_0(i)^2 - 24*a_0(i)*obj.a_max(i) - 12*obj.a_max(i)^2 - 24*obj.j_max(i)*v_0(i)), 0, 24*dir(i)*obj.j_max(i)^2*obj.a_max(i)*q_0(i) - 24*dir(i)*obj.j_max(i)^2*obj.a_max(i)*q_goal(i) + 3*a_0(i)^4 + 8*a_0(i)^3*obj.a_max(i) + 6*a_0(i)^2*obj.a_max(i)^2 + 12*a_0(i)^2*obj.j_max(i)*v_0(i) + 24*a_0(i)*obj.a_max(i)*obj.j_max(i)*v_0(i) + 12*obj.a_max(i)^2*obj.j_max(i)*v_0(i) + 12*obj.j_max(i)^2*v_0(i)^2]);
                v_drive = root(3)^2/obj.j_max(i);
                if ~imag(v_drive) && v_drive > 0
                    [t, mod_jerk_profile] = optSwitchTimes(obj, q_goal, q_0, dir*v_0, dir*a_0, v_drive);

                    % Check time constraint was fulfilled
                    if abs(t_required - t(end)) < tol
                        continue;
                    end
                end
                
                % Modified profile: Phases 2 and 6 do not exist
                root = roots([144, (-144*obj.j_max(i)*t_required + 144*a_0(i)), (72*obj.j_max(i)^2*t_required^2 - 144*a_0(i)*obj.j_max(i)*t_required - 36*a_0(i)^2 - 216*obj.j_max(i)*v_0(i)), (-144*dir(i)*obj.j_max(i)^2*q_0(i) + 144*dir(i)*obj.j_max(i)^2*q_goal(i) - 48*a_0(i)^3 - 144*a_0(i)*obj.j_max(i)*v_0(i)), (144*dir(i)*obj.j_max(i)^3*q_0(i)*t_required - 144*dir(i)*obj.j_max(i)^3*q_goal(i)*t_required + 48*a_0(i)^3*obj.j_max(i)*t_required - 144*a_0(i)*dir(i)*obj.j_max(i)^2*q_0(i) + 144*a_0(i)*dir(i)*obj.j_max(i)^2*q_goal(i) + 144*a_0(i)*obj.j_max(i)^2*v_0(i)*t_required + 6*a_0(i)^4 + 72*a_0(i)^2*obj.j_max(i)*v_0(i) + 216*obj.j_max(i)^2*v_0(i)^2), 0, 72*dir(i)^2*obj.j_max(i)^4*q_0(i)^2 - 144*dir(i)^2*obj.j_max(i)^4*q_0(i)*q_goal(i) + 72*dir(i)^2*obj.j_max(i)^4*q_goal(i)^2 + 48*a_0(i)^3*dir(i)*obj.j_max(i)^2*q_0(i) - 48*a_0(i)^3*dir(i)*obj.j_max(i)^2*q_goal(i) + 144*a_0(i)*dir(i)*obj.j_max(i)^3*q_0(i)*v_0(i) - 144*a_0(i)*dir(i)*obj.j_max(i)^3*q_goal(i)*v_0(i) - a_0(i)^6 - 6*a_0(i)^4*obj.j_max(i)*v_0(i) - 36*a_0(i)^2*obj.j_max(i)^2*v_0(i)^2 - 72*obj.j_max(i)^3*v_0(i)^3]);
                v_drive = root(4)^2/obj.j_max(i);
                if ~imag(v_drive) && v_drive > 0
                    [t, mod_jerk_profile] = optSwitchTimes(obj, q_goal, q_0, dir*v_0, dir*a_0, v_drive);

                    % Check time constraint was fulfilled
                    if abs(t_required - t(end)) < tol
                        continue;
                    end
                end
                
                % No valid solution found
                mod_jerk_profile = false;
                t = zeros(obj.DoF,7);
            end
        end
        
        function [q, t_rel] = getStopPos(obj, v_0, a_0, i)
            % GETSTOPPOS % Calculate how far a joints moves until it can be
            % stopped
            
            % Set direction opposite to maximal velocity to be reached
            if v_0*a_0 > 0
                % v and a in same direction
                dir = -sign(v_0);
            else
                if abs(v_0) > 1/2*a_0^2/obj.j_max(i)
                    dir = -sign(v_0);
                else
                    dir = -sign(a_0);
                end
            end

            % If stopping direction is negative, map to pos. direction
            if(dir < 0)
                a_0 = -a_0;
                v_0 = -v_0;
            end

            % Stop joint asap
            t_rel(i,1) = (obj.a_max(i) - a_0(i))/obj.j_max(i);
            t_rel(i,3) = obj.a_max(i)/obj.j_max(i);
            t_rel(i,2) = (- v_0 - 1/2*t_rel(i,1)*a_0(i))/obj.a_max(i) - 1/2*(t_rel(i,1) + t_rel(i,3));
            
            % Check if max acceleration is reached
            if(t_rel(i,2) < -obj.Tsample)
                t_rel(i,1) = -a_0/obj.j_max(i) + sqrt(a_0^2/(2*obj.j_max(i)^2) - v_0/obj.j_max(i));
                t_rel(i,3) = t_rel(i,1) + a_0/obj.j_max(i);
                t_rel(i,2) = 0;
            end
            
            % Calculate q
            q = v_0(i)*(t_rel(i,1) + t_rel(i,2) + t_rel(i,3)) + a_0(i)*(1/2*t_rel(i,1)^2 + t_rel(i,1)*(t_rel(i,2) + t_rel(i,3)) + 1/2*t_rel(i,3)^2) + obj.j_max(i)*(1/6*t_rel(i,1)^3 + 1/2*t_rel(i,1)^2*(t_rel(i,2) + t_rel(i,3)) - 1/6*t_rel(i,3)^3 + 1/2*t_rel(i,1)*t_rel(i,3)^2) + obj.a_max(i)*(1/2*t_rel(i,2)^2 + t_rel(i,2)*t_rel(i,3));

            % Correct direction
            q = dir(i) * q;
        end
        
        function [q_traj, v_traj, a_traj] = getTrajectories(obj, t, dir, mod_jerk_profile, q_0, v_0, a_0)
            %GETTRAJECTORIES % Calculate trajectory based on jerk switch
            % times
            
            % Length of trajectory in samples
            traj_len = zeros(obj.DoF,1);
            for i=1:obj.DoF
                traj_len(i) = ceil(t(i,7)/obj.Tsample) + 1;
            end

            for i=1:obj.DoF
                % Calculate jerks, accelerations, velocities and positions
                j_traj = zeros(1,traj_len(i));

                % Calculate jerks
                sampled_t = zeros(1,7); % Jerk switch times in samples
                sampled_t_trans = zeros(1,7); % Sample fractions for each phase
                
                % For first phase: check if breaking or accelerating
                if mod_jerk_profile
                    % Only used for time scaling
                    jerk_profile = dir(i) * obj.j_max(i) * [-1 0 1 0 -1 0 1];
                else
                    % Standard case
                    jerk_profile = dir(i) * obj.j_max(i) * [1 0 -1 0 -1 0 1];
                end
                
                % Calculate inaccuraties when sampling times
                for j = 1:7
                    sampled_t_trans(j) = mod(t(i,j),obj.Tsample);
                end             

                % Round towards phases with zero jerk
                sampled_t(1) = floor(t(i,1)/obj.Tsample);
                sampled_t(2) = ceil(t(i,2)/obj.Tsample);
                sampled_t(3) = floor(t(i,3)/obj.Tsample);
                sampled_t(4) = ceil(t(i,4)/obj.Tsample);
                sampled_t(5) = floor(t(i,5)/obj.Tsample);
                sampled_t(6) = ceil(t(i,6)/obj.Tsample);
                sampled_t(7) = floor(t(i,7)/obj.Tsample);
                
                % Calculate sampled jerk trajectory
                if(sampled_t(1) > 0)
                    j_traj(1:sampled_t(1)) = jerk_profile(1);
                end
                
                for j = 2:7
                    if(sampled_t(j) - sampled_t(j-1) > 0)
                        j_traj(sampled_t(j-1)+1:sampled_t(j)) = jerk_profile(j);
                    end
                end
                
                % Add jerk of fractioned samples
                % (only if acceleration phases exist)
                if(sampled_t(3) >= sampled_t(2))
                    j_traj(sampled_t(1) + 1) = j_traj(sampled_t(1) + 1) + sampled_t_trans(1)/obj.Tsample * jerk_profile(1);
                    if(sampled_t(2) > 0)
                        j_traj(sampled_t(2)) = j_traj(sampled_t(2)) + (1 - sampled_t_trans(2)/obj.Tsample) * jerk_profile(3);
                    end
                    j_traj(sampled_t(3) + 1) = j_traj(sampled_t(3) + 1) + sampled_t_trans(3)/obj.Tsample * jerk_profile(3);
                end
                if(sampled_t(4) > 0)
                    j_traj(sampled_t(4)) = j_traj(sampled_t(4)) + (1 - sampled_t_trans(4)/obj.Tsample) * jerk_profile(5);
                end
                j_traj(sampled_t(5) + 1) = j_traj(sampled_t(5) + 1) + sampled_t_trans(5)/obj.Tsample * jerk_profile(5);
                if(sampled_t(6) > 0)
                    j_traj(sampled_t(6)) = j_traj(sampled_t(6)) + (1 - sampled_t_trans(6)/obj.Tsample) * jerk_profile(7);
                end
                j_traj(sampled_t(7) + 1) = j_traj(sampled_t(7) + 1) + sampled_t_trans(7)/obj.Tsample * jerk_profile(7);

                % Calculate accelerations
                a_traj = obj.Tsample * cumsum(j_traj) + a_0(i);

                % Calculate velocities
                v_traj = obj.Tsample * cumsum(a_traj) + v_0(i);

                % Calculate positions
                q_traj = obj.Tsample * cumsum(v_traj) + q_0(i);
            end
        end
    end
end
