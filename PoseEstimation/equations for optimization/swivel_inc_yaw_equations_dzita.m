function J = swivel_inc_yaw_equations_dzita(zita,EQS)

    temp = SetPoseAndUpdate0KIN(EQS.MyUACDPR,zita);
    temp = ComputeJaclOrtPar(temp);
    Jsigma = temp.AnalJac.Swivel;
    Jinc_yaw = [zeros(3) eye(3)];

    J = [Jsigma; Jinc_yaw];

end