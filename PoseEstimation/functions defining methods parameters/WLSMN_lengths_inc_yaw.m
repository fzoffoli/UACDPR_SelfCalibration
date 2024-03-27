function S = WLSMN_lengths_inc_yaw(S,MyUACDPR,amperr_l,amperr_rollpitch,amperr_yaw,eps)
    
    % WLS MODIFIED NEWTON LI  
    S.Method = 'L+I MN';
    S.Key = '02';
    % Equations
    S.Equations.MyUACDPR = MyUACDPR;
    % S(j).Equations.guess.x = first_pose_guess;
    % S(j).Equations.guess.P = eye(6);
    S.Equations.F = @lengths_inc_yaw_equations;
    S.Equations.J = @lengths_inc_yaw_equations_dzita;
    S.Equations.El = amperr_l;
    S.Equations.Erollpitch = amperr_rollpitch;
    S.Equations.Eyaw = amperr_yaw;
    % Algorithm
    S.Optimization.Algorithm = @mod_newton_algorithm;
    S.Optimization.Step_Computation_Method = @weigthed_least_squares;
    S.Optimization.weigth_matrix = @weigth_matrix_lengths_inc_yaw;
    % Algorithm options
    S.Optimization.Options.Single_Equation_Tolerance =  [(1+eps)*amperr_l*ones(double(MyUACDPR.CablesNumber),1);
                                                            (1+eps)*amperr_rollpitch*ones(2,1)
                                                            (1+eps)*amperr_yaw]; 
    S.Optimization.Options.Step_Tolerance = 1e-4;
    S.Optimization.Options.Max_Function_Evaluation = 1e2;

end