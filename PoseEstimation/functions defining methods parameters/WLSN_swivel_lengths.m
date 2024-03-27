function [S,j] = WLSN_swivel_lengths(S,j,MyUACDPR,amperr_l,amperr_sigma,eps)
    % WLS SL NEWTON 
    j = j+1;
    S(j).Method = 'L+S N';
    S(j).Key = '00';
    % Equations
    S(j).Equations.MyUACDPR = MyUACDPR;
    % S(j).Equations.guess.x = first_pose_guess;
    % S(j).Equations.guess.P = eye(6);
    S(j).Equations.F = @lengths_swivel_equations;
    S(j).Equations.J = @lengths_swivel_equations_dzita;
    S(j).Equations.El = amperr_l;
    S(j).Equations.Esigma = amperr_sigma;
    % Algorithm
    S(j).Optimization.Algorithm = @newton_algorithm;
    S(j).Optimization.Step_Computation_Method = @weigthed_least_squares;
    S(j).Optimization.weigth_matrix = @weigth_matrix_lengths_swivel;
    % Algorithm options
    S(j).Optimization.Options.Function_Tolerance =  (1+eps)*sqrt(double(MyUACDPR.CablesNumber)*(amperr_l^2 + amperr_sigma^2));
    S(j).Optimization.Options.Step_Tolerance = 1e-4;
    S(j).Optimization.Options.Max_Function_Evaluation = 1e2;

end