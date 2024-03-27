function J = lengths_inc_equations_dzita(zita,EQS)

    temp = SetPoseAndUpdate0KIN(EQS.MyUACDPR,zita);
    temp = ComputeJaclOrtPar(temp);
    Jl = temp.AnalJac.Cables;
    Jinc = [zeros(2,3) eye(2) zeros(2,1)];

    J = [Jl; Jinc];

end