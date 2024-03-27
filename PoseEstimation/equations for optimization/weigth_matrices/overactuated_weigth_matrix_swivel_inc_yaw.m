function W = overactuated_weigth_matrix_swivel_inc_yaw(~,EQS,~)

    n = double(EQS.cdpr_p.n_cables);

    W_inc = diag([1/EQS.Erollpitch^2;   
              1/EQS.Erollpitch^2; 
              1/EQS.Eyaw^2]);
    
    W = [eye(n)*1/EQS.Esigma^2     zeros(n,3);
         zeros(3,n)                W_inc];
end