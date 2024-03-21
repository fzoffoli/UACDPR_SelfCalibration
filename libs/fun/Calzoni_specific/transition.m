function [out]=transition(in1,in2,t,t_transit)
persistent t0
if isempty(t0)
    t0=t;
end

a=(t-(t0))/t_transit;

if t<t0+t_transit
    out=in2*(a)+in1*(1-a);
else
    out=in2;
end