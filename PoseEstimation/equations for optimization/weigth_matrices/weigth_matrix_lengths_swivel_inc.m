function W = weigth_matrix_lengths_swivel_inc(~,EQS,~)

    n = double(EQS.MyUACDPR.CablesNumber);
    
    W = [eye(n)*1/EQS.El^2    zeros(n)                  zeros(n,2);
         zeros(n)             eye(n)*1/EQS.Esigma^2     zeros(n,2);
         zeros(2,n)           zeros(2,n)                eye(2)*1/EQS.Eeps^2];
end