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
        
        function t = optSwitchTimes(obj,q_goal, q_0, v_0, a_0)
            %OPTSWITCHTIMES Calculate time-optimal jerk swtiches

            % Set number of iteration cycles for numerc approximation
            % (only in case v_max is not reach)
            v_max_reduction_cycles = 14;
            v_max_reduced = obj.v_max*ones(obj.DoF);
            
            % Parameters for calculation
            dir = zeros(obj.DoF,1); % Direction of the goal (only +1 or -1)
            t_rel = zeros(obj.DoF,7); % Time that is required for one jerk phase
            t_rel_prev = zeros(1,7);
            t = zeros(obj.DoF,7); % Absolute time that is required to reach the end of current jerk phase
            stop = zeros(obj.DoF,1); % Exception if a goal cannot be reached -> Stop this joint asap (boolean)
            slowest_i = 0; % Number of slowest joint

            %% Analyse input data
            % Check if inputs are in limits
            checkInputs(obj, v_0, a_0);
            
            for i=1:obj.DoF
                
                % Stop joint for position equal to goal
                if(q_goal(i) == q_0(i))
                    stop(i) = 1;
                    continue
                end

                % Calculate direction of movement
                dir(i) = sign(q_goal(i) - (q_0(i) + getStopPos(obj, v_0, a_0, i)));

                % If goal is in negative direction, change v_0 and a_0
                if(dir(i) < 0)
                    v_0(i) = -v_0(i);
                    a_0(i) = -a_0(i);
                end
            end
            
            %% Calculate min. time required per joint to reach goal state
            for i=1:obj.DoF

                % Skip this joint if current joint has to be stopped or is the
                % slowest joint
                if(stop(i) || i == slowest_i)
                    continue
                end

                % Reduction loop of v_max (if necessary)
                v_max_reduction_loop_no = 0;
                while(v_max_reduction_loop_no < v_max_reduction_cycles)

                    % j = +- j_max (const) -> linear a
                    t_rel(i,1) = (obj.a_max(i) - a_0(i))/obj.j_max(i);
                    t_rel(i,3) = obj.a_max(i)/obj.j_max(i);
                    t_rel(i,5) = t_rel(i,3);
                    t_rel(i,7) = t_rel(i,3);

                    % a = +- a_max (const) -> linear v
                    % works but only if all times > 0
                    t_rel(i,2) = (v_max_reduced(i) - v_0(i) - 1/2*t_rel(i,1)*a_0(i))/obj.a_max(i) - 1/2*(t_rel(i,1) + t_rel(i,3));
                    t_rel(i,6) = v_max_reduced(i)/obj.a_max(i) - 1/2*(t_rel(i,5) + t_rel(i,7));

                    % Check if max acceleration cannot be reached
                    if(t_rel(i,2) < 0)
                        % Check if root is positive
                        root = obj.j_max(i)*(v_max_reduced(i) - v_0(i)) + 1/2*a_0(i)^2;
                        if(root > 0)
                            t_rel(i,3) = sqrt(root)/obj.j_max(i);
                            t_rel(i,1) = t_rel(i,3) - a_0(i)/obj.j_max(i);
                            t_rel(i,2) = 0;
                        else
                            if v_max_reduction_loop_no == 0
                                error("Negative root in t_rel(" + i + ",2): " + root)
                            else
                                % Approximate v_max (increase)
                                v_max_reduction_loop_no = v_max_reduction_loop_no + 1;
                                v_max_reduced(i) = v_max_reduced(i) + obj.v_max(i) * (1/2)^v_max_reduction_loop_no;
                                continue;
                            end
                        end
                    end
                    if(t_rel(i,6)<0)
                        % Check if root is positive
                        root = v_max_reduced(i)/obj.j_max(i);
                        if(root > 0)
                            t_rel(i,5) = sqrt(root);
                            t_rel(i,7) = t_rel(i,5);
                            t_rel(i,6) = 0;
                        else
                            if v_max_reduction_loop_no == 0
                                error("Negative root in t_rel(" + i + ",6): " + root)
                            else
                                % Approximate v_max (increase)
                                v_max_reduction_loop_no = v_max_reduction_loop_no + 1;
                                v_max_reduced(i) = v_max_reduced(i) + obj.v_max(i) * (1/2)^v_max_reduction_loop_no;
                                continue;
                            end
                        end
                    end

                    % v = v_max (const) -> linear q
                    q_part1 = v_0(i)*(t_rel(i,1) + t_rel(i,2) + t_rel(i,3)) + a_0(i)*(1/2*t_rel(i,1)^2 + t_rel(i,1)*(t_rel(i,2) + t_rel(i,3)) + 1/2*t_rel(i,3)^2) + obj.j_max(i)*(1/6*t_rel(i,1)^3 + 1/2*t_rel(i,1)^2*(t_rel(i,2) + t_rel(i,3)) - 1/6*t_rel(i,3)^3 + 1/2*t_rel(i,1)*t_rel(i,3)^2) + obj.a_max(i)*(1/2*t_rel(i,2)^2 + t_rel(i,2)*t_rel(i,3));
                    q_part2 = obj.j_max(i)*(1/6*t_rel(i,7)^3 + 1/2*t_rel(i,7)^2*(t_rel(i,6) + t_rel(i,5)) - 1/6*t_rel(i,5)^3 + 1/2*t_rel(i,7)*t_rel(i,5)^2) + obj.a_max(i)*(1/2*t_rel(i,6)^2 + t_rel(i,6)*t_rel(i,5));
                    t_rel(i,4) = (abs(q_goal(i) - q_0(i)) - q_part1 - q_part2)/v_max_reduced(i);

                    % Check if max velocity cannot be reached
                    if(t_rel(i,4) < 0)
                        % Formula calculated with matlab
                        % Check if root is positive
                        root = (obj.j_max(i)^2*t_rel(i,1)^4)/2 - (obj.j_max(i)^2*t_rel(i,3)^4)/4 + (obj.j_max(i)^2*t_rel(i,3)^2*t_rel(i,5)^2)/2 - (obj.j_max(i)^2*t_rel(i,5)^4)/4 + (obj.j_max(i)^2*t_rel(i,7)^4)/2 + 2*obj.j_max(i)*a_0(i)*t_rel(i,1)^3 - (2*obj.j_max(i)*obj.a_max(i)*t_rel(i,1)^3)/3 - 2*obj.j_max(i)*obj.a_max(i)*t_rel(i,1)*t_rel(i,3)^2 + (2*obj.j_max(i)*obj.a_max(i)*t_rel(i,3)^3)/3 + (2*obj.j_max(i)*obj.a_max(i)*t_rel(i,5)^3)/3 - 2*obj.j_max(i)*obj.a_max(i)*t_rel(i,5)^2*t_rel(i,7) - (2*obj.j_max(i)*obj.a_max(i)*t_rel(i,7)^3)/3 + 2*obj.j_max(i)*v_0(i)*t_rel(i,1)^2 + 2*a_0(i)^2*t_rel(i,1)^2 - 2*a_0(i)*obj.a_max(i)*t_rel(i,1)^2 - 2*a_0(i)*obj.a_max(i)*t_rel(i,3)^2 + 4*a_0(i)*v_0(i)*t_rel(i,1) + 2*obj.a_max(i)^2*t_rel(i,3)^2 + 2*obj.a_max(i)^2*t_rel(i,5)^2 - 4*obj.a_max(i)*v_0(i)*t_rel(i,1) + 4*abs(q_0(i) - q_goal(i))*obj.a_max(i) + 2*v_0(i)^2;
                        if(0 < root)
                            t_rel(i,6) = -(4*obj.a_max(i)*t_rel(i,5) - 2*root^(1/2) + obj.j_max(i)*t_rel(i,3)^2 - obj.j_max(i)*t_rel(i,5)^2 + 2*obj.j_max(i)*t_rel(i,7)^2)/(4*obj.a_max(i));
                            t_rel(i,2) = (-v_0(i) - a_0(i)*t_rel(i,1) - 1/2*obj.j_max(i)*t_rel(i,1)^2 + 1/2*obj.j_max(i)*t_rel(i,3)^2 + 1/2*obj.j_max(i)*t_rel(i,7)^2 - 1/2*obj.j_max(i)*t_rel(i,5)^2)/obj.a_max(i) - t_rel(i,3) + t_rel(i,6) + t_rel(i,5);
                            t_rel(i,4) = 0;
                        else
                            % Approximate v_max (increase)
                            v_max_reduction_loop_no = v_max_reduction_loop_no + 1;
                            v_max_reduced(i) = v_max_reduced(i) + obj.v_max(i) * (1/2)^v_max_reduction_loop_no;
                        end
                        
                        % Check if max velocity and max acceleration cannot be reached
                        if(t_rel(i,6) < 0 || t_rel(i,2) < 0) 
                            % Equations not solvable, approximate v_max (decrease)
                            v_max_reduction_loop_no = v_max_reduction_loop_no + 1;
                            v_max_reduced(i) = v_max_reduced(i) - obj.v_max(i) * (1/2)^v_max_reduction_loop_no;
                            
                            % Assign t_rel from last valid loop
                            t_rel(i,:) = t_rel_prev;
                        else
                            if v_max_reduction_loop_no == 0
                                % Analytic solution found
                                break;
                            else
                                % Approximate v_max (increase)
                                v_max_reduction_loop_no = v_max_reduction_loop_no + 1;
                                v_max_reduced(i) = v_max_reduced(i) + obj.v_max(i) * (1/2)^v_max_reduction_loop_no;
                                t_rel_prev = t_rel(i,:);
                            end
                        end
                    else
                        if v_max_reduction_loop_no == 0
                            % Analytic solution found
                            break;
                        else
                            % Approximate v_max (increase)
                            v_max_reduction_loop_no = v_max_reduction_loop_no + 1;
                            v_max_reduced(i) = v_max_reduced(i) + obj.v_max(i) * (1/2)^v_max_reduction_loop_no;
                            t_rel_prev = t_rel(i,:);
                        end
                    end
                end
            end

            % Stop if any time is negative
            for j=1:7
                if(t_rel(i,j) < 0)
                    error("t_rel(" + i + "," + j + ") is negative: " + t_rel(i,j))
                    break
                end
            end

            % Calculate absolute times for jerk switches
            t(i,:) = cumsum(t_rel(i,:));
        end
        
        function q = getStopPos(obj, v_0, a_0, i)
            % GETSTOPPOS % Calculate how far a joints moves until it can be
            % stopped
            
            % Set direction opposite to maximal velocity to be reached
            dir(i) = -sign(v_0(i) + 1/2*a_0(i)*abs(a_0(i))/obj.j_max(i));

            % If v_0 is in positive, change v_0 and a_0
            if(dir(i) < 0)
                a_0(i) = -a_0(i);
                v_0(i) = -v_0(i);
            end

            % Stop joint asap
            t_rel(i,1) = (obj.a_max(i) - a_0(i))/obj.j_max(i);
            t_rel(i,3) = obj.a_max(i)/obj.j_max(i);
            t_rel(i,2) = (- v_0(i) - 1/2*t_rel(i,1)*a_0(i))/obj.a_max(i) - 1/2*(t_rel(i,1) + t_rel(i,3));
            if(t_rel(i,2) < 0)
                % Check if root is positive, otherwise switch times
                root = obj.j_max(i)*v_0(i) - 1/2*a_0(i)*abs(a_0(i));
                if(root > 0)
                    t_rel(i,1) = sqrt(root)/obj.j_max(i);
                    t_rel(i,3) = t_rel(i,1) - a_0(i)/obj.j_max(i);
                else
                    t_rel(i,3) = sqrt(-root)/obj.j_max(i);
                    t_rel(i,1) = t_rel(i,3) - a_0(i)/obj.j_max(i);
                end
                t_rel(i,2) = 0;
            end
            
            % Calculate q
            q = v_0(i)*(t_rel(i,1) + t_rel(i,2) + t_rel(i,3)) + a_0(i)*(1/2*t_rel(i,1)^2 + t_rel(i,1)*(t_rel(i,2) + t_rel(i,3)) + 1/2*t_rel(i,3)^2) + obj.j_max(i)*(1/6*t_rel(i,1)^3 + 1/2*t_rel(i,1)^2*(t_rel(i,2) + t_rel(i,3)) - 1/6*t_rel(i,3)^3 + 1/2*t_rel(i,1)*t_rel(i,3)^2) + obj.a_max(i)*(1/2*t_rel(i,2)^2 + t_rel(i,2)*t_rel(i,3));

            % Correct direction
            q = dir(i) * q;
        end 
    end
end
