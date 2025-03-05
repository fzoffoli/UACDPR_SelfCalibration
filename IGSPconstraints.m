function [c,ceq]=IGSPconstraints(MyUACDPR,position,tau_eps)

c=[];
ceq=IGSPsolverBrutal(MyUACDPR,position,tau_eps);

end