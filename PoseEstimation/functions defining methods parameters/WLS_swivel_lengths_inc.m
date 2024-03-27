function [S,j] = WLS_swivel_lengths_inc(S,j,MyUACDPR,amperr_l,amperr_sigma,amperr_eps,eps)

    % WLS NEWTON SLI  
    j = j+1;
    S(j).Method = 'L+S+I N';
    S(j).Key = '01';
    % Equations
    S(j).Equations.MyUACDPR = MyUACDPR;
    % S(j).Equations.guess.x = first_pose_guess;
    % S(j).Equations.guess.P = eye(6);
    S(j).Equations.F = @lengths_swivel_inc_equations;
    S(j).Equations.J = @lengths_swivel_inc_equations_dzita;
    S(j).Equations.El = amperr_l;
    S(j).Equations.Esigma = amperr_sigma;
    S(j).Equations.Eeps = amperr_eps;
    % Algorithm
    S(j).Optimization.Algorithm = @newton_algorithm;
    S(j).Optimization.Step_Computation_Method = @weigthed_least_squares;
    S(j).Optimization.weigth_matrix = @weigth_matrix_lengths_swivel_inc;
    % Algorithm options
    S(j).Optimization.Options.Function_Tolerance =  (1+eps)*sqrt(double(MyUACDPR.CablesNumber)*(amperr_l^2 + amperr_sigma^2) + 2*amperr_eps^2);
    S(j).Optimization.Options.Step_Tolerance = 1e-4;
    S(j).Optimization.Options.Max_Function_Evaluation = 1e2;
    
    % WLS MODIFIED NEWTON SLI  
    j = j+1;
    S(j).Method = 'L+S+I MN';
    S(j).Key = '01';
    % Equations
    S(j).Equations.MyUACDPR = MyUACDPR;
    % S(j).Equations.guess.x = first_pose_guess;
    % S(j).Equations.guess.P = eye(6);
    S(j).Equations.F = @lengths_swivel_inc_equations;
    S(j).Equations.J = @lengths_swivel_inc_equations_dzita;
    S(j).Equations.El = amperr_l;
    S(j).Equations.Esigma = amperr_sigma;
    S(j).Equations.Eeps = amperr_eps;
    % Algorithm
    S(j).Optimization.Algorithm = @mod_newton_algorithm;
    S(j).Optimization.Step_Computation_Method = @weigthed_least_squares;
    S(j).Optimization.weigth_matrix = @weigth_matrix_lengths_swivel_inc;
    % Algorithm options
    S(j).Optimization.Options.Single_Equation_Tolerance =  [(1+eps)*amperr_l*ones(double(MyUACDPR.CablesNumber),1);
                                                            (1+eps)*amperr_sigma*ones(double(MyUACDPR.CablesNumber),1)
                                                            (1+eps)*amperr_eps*ones(2,1)]; 
    S(j).Optimization.Options.Step_Tolerance = 1e-4;
    S(j).Optimization.Options.Max_Function_Evaluation = 1e2;

end