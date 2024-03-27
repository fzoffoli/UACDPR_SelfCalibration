function J = lengths_swivel_equations_dzita(zita,EQS)

    temp = SetPoseAndUpdate0KIN(EQS.MyUACDPR,zita);
    temp = ComputeJaclOrtPar(temp);
    Jl = temp.AnalJac.Cables;
    Jsigma = temp.AnalJac.Swivel;

    J = [Jl; Jsigma];

end