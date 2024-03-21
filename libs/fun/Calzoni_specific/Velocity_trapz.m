function [length_act,length_dt,length_ddt] = Velocity_trapz(t_start,t_acc,t_dec,Vmax,length0,final_length,t)
%works only for trapezoidal velocity laws
t0=t_start;
t1=t_start+t_acc;
delta_length=final_length-length0;
delta_t=(delta_length-t_acc*Vmax/2-t_dec*Vmax/2)/Vmax; 
t2=t1+delta_t;
t3=t2+t_dec;



if t<=t0 %static
    length_ddt=0;
    length_dt=0;
    length_act=length0;

elseif (t<=t1) && (t>t0) %acc cost
    length_ddt=Vmax/t_acc;
    length_dt=length_ddt*(t-t0);
    length_act=length0+.5*length_ddt*(t-t0)^2;

elseif (t1<=t) && (t<t2) %V cost
   length_ddt=0;
   length_dt=Vmax;
   length_act=length0+.5*Vmax*(t1-t0)+length_dt*(t-t1);
   
elseif (t2<=t)&&(t<=t3) %dec cost
    length_ddt=-Vmax/t_dec;
    length_dt=Vmax+length_ddt*(t-t2);
    length_act=length0+.5*Vmax*(t1-t0)+Vmax*(t-t1)+.5*length_ddt*(t-t2)^2;
else %static
    length_ddt=0;
    length_dt=0;
    length_act=length0+.5*Vmax*(t1-t0)+Vmax*(t2-t1)+.5*Vmax*(t3-t2);
end
end

