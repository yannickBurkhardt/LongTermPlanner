% Define test values
eps = 0.001;
num_scenarios = 9;
v_max  = [  2       2       2       1       1       8       8       8       8       ];
a_max  = [  2       2       2       2       2       2       2       2       2       ];
j_max  = [  4       4       4       4       4       4       4       4       4       ];
q_goal = [ -1       2.927   2.8854  0.2396  0.6354  1.927   1.8854 -0.2604  0.1354  ];
q_0    = -ones(9);
v_0    = [  0       0.625   1.875  -0.875   0.875   0.625   1.875  -0.875   0.875   ];
a_0    = [  0       1      -1      1       -1       1      -1       1       -1      ];

% Define pre-calculated results
t_rel  = [  0       0.25    0.5     0.25    0.5     0.25    0.5     0.25    0.5     ;
            0       0.25    0       0.5     0       0.25    0       0.5     0       ;
            0       0.5     0.25    0.5     0.25    0.5     0.25    0.5     0.25    ;
            0       0.5     0.5     0.5     0.5     0       0       0       0       ;
            0       0.5     0.5     0.5     0.5     0.5     0.5     0.5     0.5     ;
            0       0.5     0.5     0       0       0.5     0.5     0       0       ;
            0       0.5     0.5     0.5     0.5     0.5     0.5     0.5     0.5     ]';
t = cumsum(t_rel,2);

% Initialize Planner
ltp = LTPlanner(1, 0.001);

% Test all scenarios
sucess = 0;
fail = 0;
for i=1:num_scenarios
    % Set limtits
    ltp.setLimits(v_max(i), a_max(i), j_max(i));
    
    % Compare times to pre-calculation
    t_ltp = ltp.optSwitchTimes(q_goal(i), q_0(i), v_0(i), a_0(i));
    if(all(abs(t_ltp - t(i,:)) < eps))
        sucess = sucess + 1;
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
    t_ltp = ltp.optSwitchTimes(-q_goal(i), -q_0(i), -v_0(i), -a_0(i));
    if(all(abs(t_ltp - t(i,:)) < eps))
        sucess = sucess + 1;
    else
        disp("Failure in test " + i + ".2.")
        t(i,:)
        t_ltp
        fail = fail + 1;
    end
end

% Print test results
disp("TestOptSwitchTimes results:")
disp("Sucessful: " + sucess + " out of " + (2 * num_scenarios - 1))
disp("Failure:   " + fail + " out of " + (2 * num_scenarios - 1))
