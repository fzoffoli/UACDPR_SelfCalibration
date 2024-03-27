function [S,j] = WLSN_swivel_inc(S,j,MyUACDPR,amperr_sigma,amperr_eps,eps)

    % WLS NEWTON SI  
    j = j+1;
    S(j).Method = 'S+I N';
    S(j).Key = '03';
    % Equations
    S(j).Equations.MyUACDPR = MyUACDPR;
    % S(j).Equations.guess.x = first_pose_guess;
    % S(j).Equations.guess.P = eye(6);
    S(j).Equations.F = @swivel_inc_equations;
    S(j).Equations.J = @swivel_inc_equations_dzita;
    S(j).Equations.Esigma = amperr_sigma;
    S(j).Equations.Eeps = amperr_eps;
    % Algorithm
    S(j).Optimization.Algorithm = @newton_algorithm;
    S(j).Optimization.Step_Computation_Method = @weigthed_least_squares;
    S(j).Optimization.weigth_matrix = @weigth_matrix_swivel_inc;
    % Algorithm options
    S(j).Optimization.Options.Function_Tolerance =  (1+eps)*sqrt(double(MyUACDPR.CablesNumber)*amperr_sigma^2 + 2*amperr_eps^2);
    S(j).Optimization.Options.Step_Tolerance = 1e-4;
    S(j).Optimization.Options.Max_Function_Evaluation = 1e2;
    
end