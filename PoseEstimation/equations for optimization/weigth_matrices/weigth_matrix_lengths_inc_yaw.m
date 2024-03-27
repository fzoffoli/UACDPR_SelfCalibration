function W = weigth_matrix_lengths_inc_yaw(~,EQS,~)

    n = double(EQS.MyUACDPR.CablesNumber);

    W_inc = diag([1/EQS.Erollpitch^2;   
                  1/EQS.Erollpitch^2; 
                  1/EQS.Eyaw^2]);
    
    W = [eye(n)*1/EQS.El^2    zeros(n,3);
         zeros(3,n)           W_inc];
end