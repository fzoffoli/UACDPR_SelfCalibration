function [err_p]=simple_error_integration(x,contr2)
e_newddot=-(contr2.k1+contr2.Gamma)*x(16:18)-contr2.k1*contr2.Gamma*x(7:9)-contr2.k2*satur(x(16:18)+contr2.Gamma*x(7:9));
e_newdot=x(16:18);

err_p.pos=x(7:9);
err_p.vel=e_newdot;
err_p.acc=e_newddot;
end