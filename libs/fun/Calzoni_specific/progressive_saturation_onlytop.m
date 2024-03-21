function tau = progressive_saturation_onlytop(tau_max,tau_min,tau)
for i=1:length(tau)
    if tau(i)<tau_min(i)
        tau(i)=tau_min(i);
    else
        k=-1/(tau_min(i)/tau_max(i)-1)-tau_min(i)/tau_max(i);
        tau(i)=tau_max(i)*(-1/(tau(i)/10000+k)+1);
    end
end

end