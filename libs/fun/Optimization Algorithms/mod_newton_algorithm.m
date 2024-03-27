function [result,output] = mod_newton_algorithm(EQS,OPT)

    % EQS.guess.x: starting vector
    % EQS.guess.P: covariance of the starting vector (UNUSED)
    % EQS.F: function handle to minimization function
    % EQS.J: function handle to dF/dx 
    % OPT.Step_Computation_Method: function handle to step computation function
    % OPT.Options: algorithm options  
    
    x = EQS.guess.x;
    P = EQS.guess.P;
    dx = OPT.Options.Step_Tolerance;
    k = 0;
    Toll_not_satisfied = any(abs(EQS.F(x,EQS))>OPT.Options.Single_Equation_Tolerance);
    
    while (Toll_not_satisfied)&&(norm(dx)>=OPT.Options.Step_Tolerance)&&...
    (k<OPT.Options.Max_Function_Evaluation)
        k = k+1;
        [dx,P] = OPT.Step_Computation_Method(x,EQS,OPT);
       % x = x+dx;         
       x = x+dx;    
        Toll_not_satisfied = any(abs(EQS.F(x,EQS))>OPT.Options.Single_Equation_Tolerance);
    end

    result.x = x;
    result.P = P;
    output.Residual = norm(EQS.F(x,EQS));
    output.Stepsize = norm(dx);
    output.Iterations = k;
end

