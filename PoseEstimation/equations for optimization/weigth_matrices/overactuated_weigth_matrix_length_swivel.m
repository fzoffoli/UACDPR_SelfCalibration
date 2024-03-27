function W = overactuated_weigth_matrix_length_swivel(~,EQS,~)

    n = double(EQS.cdpr_p.n_cables);
    
    W = [eye(n)*1/EQS.El^2    zeros(n)             ;
         zeros(n)             eye(n)*1/EQS.Esigma^2];
end