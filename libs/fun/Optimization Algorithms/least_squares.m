function [dx,P] = least_squares(x,EQS,~)
    
    F = EQS.F(x,EQS);
    J = EQS.J(x,EQS);
    P = (J'*J)^(-1);
    dx = -linsolve(J'*J,J'*F);
end