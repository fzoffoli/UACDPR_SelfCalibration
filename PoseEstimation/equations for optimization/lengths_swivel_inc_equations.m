function F = lengths_swivel_inc_equations(zita,EQS)

    temp = SetPoseAndUpdate0KIN(EQS.MyUACDPR,zita);
    lengths = temp.CableLengths_;
    n = double(EQS.MyUACDPR.CablesNumber);
    sigma = zeros(n,1);
    for j = 1:n
        sigma(j,1) = temp.Trasmission.Pulley{j}.SwivelAngle;            
    end
    eps = zita(4:5);

    F = [lengths; sigma; eps] - EQS.measures;
end
