function J = lengths_swivel_inc_yaw_equations_dzita(zita,EQS)

    temp = SetPoseAndUpdate0KIN(EQS.MyUACDPR,zita);
    temp = ComputeJaclOrtPar(temp);
    Jl = temp.AnalJac.Cables;
    Jsigma = temp.AnalJac.Swivel;
    Jinc_yaw = [zeros(3,3) eye(3)];

    J = [Jl; Jsigma; Jinc_yaw];

end