function tau = banal_saturation(tau_max,tau_min,tau)
    for i=1:length(tau)
        if tau(i)>tau_max(i)
            tau(i)=tau_max(i);
        elseif tau(i)<tau_min
            tau(i)=tau_min(i);
        end
    end