function [S,j] = WLSMN_lengths_inc(S,j,MyUACDPR,amperr_l,amperr_eps,eps)
    
    % WLS MODIFIED NEWTON LI  
    j = j+1;
    S(j).Method = 'L+I MN';
    S(j).Key = '02';
    % Equations
    S(j).Equations.MyUACDPR = MyUACDPR;
    % S(j).Equations.guess.x = first_pose_guess;
    % S(j).Equations.guess.P = eye(6);
    S(j).Equations.F = @lengths_inc_equations;
    S(j).Equations.J = @lengths_inc_equations_dzita;
    S(j).Equations.El = amperr_l;
    S(j).Equations.Eeps = amperr_eps;
    % Algorithm
    S(j).Optimization.Algorithm = @mod_newton_algorithm;
    S(j).Optimization.Step_Computation_Method = @weigthed_least_squares;
    S(j).Optimization.weigth_matrix = @weigth_matrix_lengths_inc;
    % Algorithm options
    S(j).Optimization.Options.Single_Equation_Tolerance =  [(1+eps)*amperr_l*ones(double(MyUACDPR.CablesNumber),1);
                                                            (1+eps)*amperr_eps*ones(2,1)]; 
    S(j).Optimization.Options.Step_Tolerance = 1e-4;
    S(j).Optimization.Options.Max_Function_Evaluation = 1e2;

end