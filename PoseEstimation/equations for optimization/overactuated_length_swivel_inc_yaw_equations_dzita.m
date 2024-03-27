function J = overactuated_length_swivel_inc_yaw_equations_dzita(zita,EQS)

    cdpr_v = UpdateIKZeroOrd(zita(1:3),zita(4:6),EQS.cdpr_p,EQS.cdpr_v);
    Jl = compute_JacobianLength(cdpr_v);
    Jsigma = compute_JacobianSwivel(cdpr_v);
    Jinc_yaw = [zeros(3) eye(3)];

    J = [Jl; Jsigma; Jinc_yaw];

end