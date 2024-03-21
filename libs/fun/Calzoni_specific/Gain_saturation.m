function [gain,tau0] = Gain_saturation(tau_max,tau_min,tau,gain0,tau0)
vect_k=ones(length(tau),1);
for i=1:length(tau)
if tau(i) > tau0
vect_k(i)=min((tau_max(i)-tau0)./(tau(i)-tau0));
else
vect_k(i)=min((tau0-tau_min(i))./(tau0-tau(i)));
end
end

factor_gain=min(vect_k);
if factor_gain==0
    factor_gain=0;
elseif factor_gain<0
    factor_gain=0;
    tau0=min(tau_max);
elseif factor_gain>=1
    factor_gain=1;
end

gain=gain0*factor_gain;

end