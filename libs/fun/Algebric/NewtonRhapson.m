function [r,iter] = NewtonRhapson(Fun,Jac,x0,xtol,ftol,nmax)
% Questa funzione implementa il metodo di Newton per sistemi di equazioni
% non lineari. Fun e J rappresentano la funzione e il suo jacobiano, xtol e % ftol le tolleranze sullo zero e sul residuo, maxit il numero massimo di
% iterazioni, x0 il punto di partenza.
% r è il vettore che contiene l'approssimazione dello zero, iter il numero % di iterazioni eseguite ed incr è il vettore degli incrementi.
 
  iter = 0;
  x = x0;
  err = 2;
  condizione = 1; Fxn = Fun(x0); 
  
  while iter < nmax && norm(Fxn) > ftol && err > condizione
    
    iter = iter + 1;
    Jxn = Jac(x);
    s = Jxn\Fxn;
    x = x - s;
    Fxn = Fun(x);
    err = norm(s);
    condizione = xtol*(1+ norm(x));
    
  end
    
  r = x;
  
end