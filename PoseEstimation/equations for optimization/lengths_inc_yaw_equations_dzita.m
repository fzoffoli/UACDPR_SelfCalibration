function J = lengths_inc_yaw_equations_dzita(zita,EQS)

    temp = SetPoseAndUpdate0KIN(EQS.MyUACDPR,zita);
    temp = ComputeJaclOrtPar(temp);
    Jl = temp.AnalJac.Cables;
    Jinc_yaw = [zeros(3) eye(3)];

    J = [Jl; Jinc_yaw];

end