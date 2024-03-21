function bfdx = BFD(o,x,n0)    %Backward finite difference
     % o = ordine di accuratezza della derivata (1-primo ordine, ...)
     % x = vettore contenente tutti i punti della funzione
     % n0 = punto del vettore in cui calcolare la derivata
     
     % Calcolo coefficienti per BFD di ordine di accuratezza o
     t = -double(o);
     s=t:1:0;
     
     A = zeros(o+1);
     for i=1:(o+1)
         for j = 1:(o+1)
            A(i,j)=s(j)^(i-1);
         end
     end
     delta = zeros((o+1),1);
     delta(2,1) = 1;
     k = A^(-1)*delta;
     
     %Calcolo derivata di ordine di accuratezza o, nel punto x0
     x_punti = x(:,((n0-o):n0));
     bfdx = x_punti*k;   
     
end

