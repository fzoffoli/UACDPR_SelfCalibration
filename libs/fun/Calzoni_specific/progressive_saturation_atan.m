function tau_out = progressive_saturation_atan(tau_max,tau_min,tau)
for i=1:length(tau)
    mids=(tau_max(i)+tau_min(i))/2;
tau_out(i)=atan(tau(i)/5-mids)*(mids-tau_min(i))/pi*2+mids;
end
x1min=tau_min-1000;
x2min=tau_min+1000;
y1min=90;
y2min=180;
m_min(i)=(y1min-y2min)/(x1min-x2min);
q_min(i)=(x1min*y2min-x2min*y1min)/(x1min-x2min);

x1max=tau_max-1000;
x2max=tau_max+1000;
y1max=180;
y2max=270;
m_max(i)=(y1max-y2max)/(x1max-x2max);
q_max(i)=(x1max*y2max-x2max*y1max)/(x1max-x2max);


if tau(i)<tau_min(i)-1000
    tau_out(i)=tau_min(i);
elseif tau(i)<tau_min(i)+1000 && tau(i)>tau_min(i)-1000
    tau_out(i)=1000*sin(tau(i)*m_min(i)-q_min(i));
elseif tau(i)>tau_max(i)-1000 && tau(i)<tau_min(i)+1000
    tau_out(i)=1000*sin(tau(i)*m_max(i)-q_max(i));
elseif tau(i)>tau_max(i)+1000
    tau_out(i)=tau_max(i);
else
    tau_out(i)=tau(i);
end

end