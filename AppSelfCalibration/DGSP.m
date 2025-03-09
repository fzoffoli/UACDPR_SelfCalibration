function F = DGSP(MyUACDPR,zeta_lengths,tau)
    MyUACDPR = SetPoseAndUpdate0KIN(MyUACDPR,zeta_lengths(1:6));
    MyUACDPR = ComputeGravityWrench(MyUACDPR);
    
    F(1:4) = zeta_lengths(7:end)-MyUACDPR.Trasmission.CableLengths;
    F(5:10) = -MyUACDPR.GeomJac.Cables'*tau+MyUACDPR.Wrench;
end