function [dx,P] = weigthed_least_squares(x,EQS,OPT)
    
%     FilmDrawRobot(EQS.MyUACDPR.PermutMatrix*(x),EQS.MyUACDPR);
    F = EQS.F(x,EQS);
    J = EQS.J(x,EQS);
    W = OPT.weigth_matrix(x,EQS,OPT);
    P = (J'*W*J)^(-1);
    dx = -linsolve(J'*W*J,J'*W*F);
end