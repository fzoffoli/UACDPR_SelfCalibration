function F = swivel_inc_equations(zita,EQS)

    temp = SetPoseAndUpdate0KIN(EQS.MyUACDPR,zita);
    n = double(EQS.MyUACDPR.CablesNumber);
    sigma = zeros(n,1);
    for j = 1:n
        sigma(j,1) = temp.Trasmission.Pulley{j}.SwivelAngle;            
    end
    eps = zita(4:5);

    F = [sigma; eps] - EQS.measures;
end
