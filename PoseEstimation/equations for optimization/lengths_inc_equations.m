function F = lengths_inc_equations(zita,EQS)

    temp = SetPoseAndUpdate0KIN(EQS.MyUACDPR,zita);
    lengths = temp.CableLengths_;
    eps = zita(4:5);

    F = [lengths; eps] - EQS.measures;
end
