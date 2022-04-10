% Define test values
eps = 0.002;
num_scenarios = 10;

% v_max increased compared to testOptSwitchTimes
v_max  = [  4       4       4       4       4       4       4       4       4       4       ];
a_max  = [  2       2       2       2       2       2       2       4       4       4       ];
j_max  = [  4       4       4       4       4       4       4       4       4       2       ];
q_goal = [ -1       2.927   2.8854  0.2396  0.6354 -7.0104 -8.9896 -5.1746 -6.6538 -8.4167  ];
q_0    = -ones(10);
v_0    = [  0       0.625   1.875  -0.875   0.875  -3.875  -3.875  -2.875  -2.875  -1.5     ];
a_0    = [  0       1      -1      1       -1       1      -1       1      -1      -2       ];

% Define pre-calculated results
t_rel  = [  0       0.25    0.5     0.25    0.5     0.25    0.75    0.25    0.75    1.5     ;
            0       0.25    0       0.5     0       0.5     0.5     0       0       0       ;
            0       0.5     0.25    0.5     0.25    0.5     0.5     0.5     0.5     0.5     ;
            0       0.5     0.5     0.5     0.5     0.5     0.5     0.5     0.5     0.5     ;
            0       0.5     0.5     0.5     0.5     0.5     0.5     0.7071  0.7071  1.0     ;
            0       0.5     0.5     0       0       0.5     0.5     0       0       0       ;
            0       0.5     0.5     0.5     0.5     0.5     0.5     0.7071  0.7071  1.0     ]';
t = cumsum(t_rel,2);

% Absolute time used for scaling
t_required = t(:,end);

% Initialize Planner
ltp = LTPlanner(1, 0.001);

% Test all scenarios
success = 0;
fail = 0;
for i=1:num_scenarios
    % Set limtits
    ltp.setLimits(v_max(i), a_max(i), j_max(i));
    
    % Compare times to pre-calculation
    t_ltp = ltp.timeScaling(q_goal(i), q_0(i), v_0(i), a_0(i), t_required(i));
    if(all(abs(t_ltp - t(i,:)) < eps))
        success = success + 1;
    else
        disp("Failure in test " + i + ".1.")
        t(i,:)
        t_ltp
        fail = fail + 1;
    end
    
    % Skip for first test
    if i == 1
        continue
    end

    % Execute same scenario in opposite direction
    t_ltp = ltp.timeScaling(q_goal(i), q_0(i), v_0(i), a_0(i), t_required(i));
    if(all(abs(t_ltp - t(i,:)) < eps))
        success = success + 1;
    else
        disp("Failure in test " + i + ".2.")
        t(i,:)
        t_ltp
        fail = fail + 1;
    end
end

% Print test results
disp("TestTimeScaling results:")
disp("Sucessful: " + success + " out of " + (2 * num_scenarios - 1))

% Throw error if at least one test failed
if fail > 0
    error("Failure:   " + fail + " out of " + (2 * num_scenarios - 1))
end
