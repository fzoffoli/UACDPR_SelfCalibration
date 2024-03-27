function J = swivel_inc_equations_dzita(zita,EQS)

    temp = SetPoseAndUpdate0KIN(EQS.MyUACDPR,zita);
    temp = ComputeJaclOrtPar(temp);
    Jsigma = temp.AnalJac.Swivel;
    Jinc = [zeros(2,3) eye(2) zeros(2,1)];

    J = [Jsigma; Jinc];

end