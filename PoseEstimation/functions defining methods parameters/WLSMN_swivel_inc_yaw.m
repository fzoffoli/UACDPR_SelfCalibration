function [S,j] = WLSMN_swivel_inc_yaw(S,j,MyUACDPR,amperr_sigma,amperr_rollpitch,amperr_yaw,eps)
    
    % WLS MODIFIED NEWTON SLI  
    j = j+1;
    S(j).Method = 'S+I MN';
    S(j).Key = '03';
    % Equations
    S(j).Equations.MyUACDPR = MyUACDPR;
    % S(j).Equations.guess.x = first_pose_guess;
    % S(j).Equations.guess.P = eye(6);
    S(j).Equations.F = @swivel_inc_yaw_equations;
    S(j).Equations.J = @swivel_inc_yaw_equations_dzita;
    S(j).Equations.Esigma = amperr_sigma;
    S(j).Equations.Erollpitch = amperr_rollpitch;
    S(j).Equations.Eyaw = amperr_yaw;
    % Algorithm
    S(j).Optimization.Algorithm = @mod_newton_algorithm;
    S(j).Optimization.Step_Computation_Method = @weigthed_least_squares;
    S(j).Optimization.weigth_matrix = @weigth_matrix_swivel_inc_yaw;
    % Algorithm options
    S(j).Optimization.Options.Single_Equation_Tolerance =  [(1+eps)*amperr_sigma*ones(double(MyUACDPR.CablesNumber),1)
                                                            (1+eps)*amperr_rollpitch*ones(2,1);
                                                            (1+eps)*amperr_yaw]; 
    S(j).Optimization.Options.Step_Tolerance = 1e-4;
    S(j).Optimization.Options.Max_Function_Evaluation = 1e2;

end