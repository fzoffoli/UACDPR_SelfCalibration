function W = weigth_matrix_lengths_swivel(~,EQS,~)

    n = double(EQS.MyUACDPR.CablesNumber);
    
    W = [eye(n)*1/EQS.El^2    zeros(n);
         zeros(n)             eye(n)*1/EQS.Esigma^2];
end