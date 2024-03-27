function W = weigth_matrix_swivel_inc(~,EQS,~)

    n = double(EQS.MyUACDPR.CablesNumber);
    
    W = [eye(n)*1/EQS.Esigma^2     zeros(n,2);
         zeros(2,n)                eye(2)*1/EQS.Eeps^2];
end