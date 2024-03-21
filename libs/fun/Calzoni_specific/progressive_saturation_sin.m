function tau_out=progressive_saturation_sin(tau_max,tau_min,tau)

tau_out=zeros(length(tau),1);
for i=1:length(tau)
    cost=(tau_max(i)-tau_min(i))/3;
    x1min=tau_min(i)-cost;
    x2min=tau_min(i)+cost;
    y1min=pi*3/2;
    y2min=pi*2;
    m_min(i)=(y1min-y2min)/(x1min-x2min);
    q_min(i)=(x1min*y2min-x2min*y1min)/(x1min-x2min);
    
    x1max=tau_max(i)-cost;
    x2max=tau_max(i)+cost;
    y1max=0;
    y2max=1/2*pi;
    m_max(i)=(y1max-y2max)/(x1max-x2max);
    q_max(i)=(x1max*y2max-x2max*y1max)/(x1max-x2max);
    
    
    if tau(i)<=tau_min(i)-cost
        tau_out(i)=tau_min(i);
    elseif tau(i)<tau_min(i)+cost && tau(i)>tau_min(i)-cost
        tau_out(i)=cost*sin(tau(i)*m_min(i)+q_min(i))+(tau_min(i)+cost);
    elseif tau(i)>tau_max(i)-cost && tau(i)<tau_max(i)+cost
        tau_out(i)=cost*sin(tau(i)*m_max(i)+q_max(i))+(tau_max(i)-cost);
    elseif tau(i)>=tau_max(i)+cost
        tau_out(i)=tau_max(i);
    else
        tau_out(i)=tau(i);
    end
end
end