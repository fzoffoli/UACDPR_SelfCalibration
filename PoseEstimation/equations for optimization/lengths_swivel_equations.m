function F = lengths_swivel_equations(zita,EQS)

    temp = SetPoseAndUpdate0KIN(EQS.MyUACDPR,zita);
    lengths = temp.CableLengths_;
    n = double(EQS.MyUACDPR.CablesNumber);
    sigma = zeros(n,1);
    for j = 1:n
        sigma(j,1) = temp.Trasmission.Pulley{j}.SwivelAngle;            
    end

    F = [lengths; sigma] - EQS.measures;
end
