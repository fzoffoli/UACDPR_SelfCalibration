function F = overactuated_swivel_inc_yaw_equations(zita,EQS)

    cdpr_v = UpdateIKZeroOrd(zita(1:3),zita(4:6),EQS.cdpr_p,EQS.cdpr_v);    
    n = double(EQS.cdpr_p.n_cables);
    sigma = zeros(n,1);
    for j = 1:n        
        sigma(j,1) = cdpr_v.cable(j,1).swivel_ang;           
    end
    eps = zita(4:6);

    F = [sigma; eps] - EQS.measures;
end
