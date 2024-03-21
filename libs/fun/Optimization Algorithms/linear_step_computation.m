function [dx,P] = linear_step_computation(x,EQS,~)

    F = EQS.F(x,EQS);
    J = EQS.J(x,EQS);
    dx = -(J)^(-1)*F;
    P = EQS.guess.P;
end