% Define test values
eps = 0.01;
num_scenarios = 6;
v_max  = 10;
a_max  = [  2       2       2       4       4       4       ];
j_max  = [  4       4       4       4       4       2       ];
v_0    = [  0      -1.875  -1.875  -0.875  -0.875   0.5     ];
a_0    = [  0       1      -1       1      -1       -2      ];

% Define pre-calculated results
q_goal = [  0      -1.0104 -1.9896 -0.2604 -0.7396 -0.4167  ];
t_rel  = [  0       0.25    0.75    0.25    0.75    1.5     ;
            0       0.5     0.5     0       0       0       ;
            0       0.5     0.5     0.5     0.5     0.5     ]';

% Initialize Planner
ltp = LTPlanner(1, 0.001);

% Test all scenarios
success = 0;
fail = 0;
for i=1:num_scenarios
    % Set limtits
    ltp.setLimits(v_max, a_max(i), j_max(i));
    
    % Compare times to pre-calculation
    [q_ltp, t_ltp] = ltp.getStopPos(v_0(i), a_0(i), 1);
    if(all(abs(t_ltp - t_rel(i,:)) < eps) && (abs(q_ltp - q_goal(i)) < eps))
        success = success + 1;
    else
        disp("Failure in test " + i + ".1.")
        t_rel(i,:)
        t_ltp
        q_ltp
        q_goal(i)
        fail = fail + 1;
    end
    
    % Skip for first test
    if i == 1
        continue
    end

    % Execute same scenario in opposite direction
    [q_ltp, t_ltp] = ltp.getStopPos(-v_0(i), -a_0(i), 1);;
    if(all(abs(t_ltp - t_rel(i,:)) < eps) && (abs(q_ltp + q_goal(i)) < eps))
        success = success + 1;
    else
        disp("Failure in test " + i + ".2.")
        t_rel(i,:)
        t_ltp
        q_ltp
        -q_goal(i)
        fail = fail + 1;
    end
end

% Print test results
disp("TestGetStopPos results:")
disp("Successful: " + success + " out of " + (2 * num_scenarios - 1))

% Throw error if at least one test failed
if fail > 0
    error("Failure:   " + fail + " out of " + (2 * num_scenarios - 1))
end
