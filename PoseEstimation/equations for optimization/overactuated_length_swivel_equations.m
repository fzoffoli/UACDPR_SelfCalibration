function F = overactuated_length_swivel_equations(zita,EQS)

    cdpr_v = UpdateIKZeroOrd(zita(1:3),zita(4:6),EQS.cdpr_p,EQS.cdpr_v);    
    n = double(EQS.cdpr_p.n_cables);
    l = cdpr_v.cable_vector;
    sigma = zeros(n,1);
    for j = 1:n        
        sigma(j,1) = cdpr_v.cable(j,1).swivel_ang;           
    end

    F = [l; sigma] - EQS.measures;
end
