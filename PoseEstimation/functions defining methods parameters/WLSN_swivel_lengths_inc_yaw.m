function [S,j] = WLSN_swivel_lengths_inc_yaw(S,j,MyUACDPR,amperr_l,amperr_sigma,amperr_rollpitch,amperr_yaw,eps)

    % WLS NEWTON SLI  
    j = j+1;
    S(j).Method = 'Alg. A   $T_{\|r\|}=10^{-2}$';
    S(j).Key = '01';
    % Equations
    S(j).Equations.MyUACDPR = MyUACDPR;
    % S(j).Equations.guess.x = first_pose_guess;
    % S(j).Equations.guess.P = eye(6);
    S(j).Equations.F = @lengths_swivel_inc_yaw_equations;
    S(j).Equations.J = @lengths_swivel_inc_yaw_equations_dzita;
    S(j).Equations.El = amperr_l;
    S(j).Equations.Esigma = amperr_sigma;
    S(j).Equations.Erollpitch = amperr_rollpitch;
    S(j).Equations.Eyaw = amperr_yaw;
    % Algorithm
    S(j).Optimization.Algorithm = @newton_algorithm;
    S(j).Optimization.Step_Computation_Method = @weigthed_least_squares;
    S(j).Optimization.weigth_matrix = @weigth_matrix_lengths_swivel_inc_yaw;
    % Algorithm options
%     S(j).Optimization.Options.Function_Tolerance =  (1+eps)*sqrt(double(MyUACDPR.CablesNumber)*(amperr_l^2 + amperr_sigma^2) + ...
%         2*amperr_rollpitch^2 + amperr_yaw^2);
    S(j).Optimization.Options.Function_Tolerance = 1e-2;  
    S(j).Optimization.Options.Step_Tolerance = 1e-4;
    S(j).Optimization.Options.Max_Function_Evaluation = 1e2;
    
end