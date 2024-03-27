function J = overactuated_length_swivel_equations_dzita(zita,EQS)

    cdpr_v = UpdateIKZeroOrd(zita(1:3),zita(4:6),EQS.cdpr_p,EQS.cdpr_v);
    Jl = compute_JacobianLength(cdpr_v);
    Jsigma = compute_JacobianSwivel(cdpr_v);

    J = [Jl; Jsigma];

end