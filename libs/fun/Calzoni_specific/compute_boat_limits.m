function points=compute_boat_limits(L,W,H,PtoG,discret)


discrete_L=-L/2:discret:L/2;
discrete_H=-H/2:discret:H/2;
discrete_W=-W/2:discret:W/2;

N_L=length(discrete_L);
N_H=length(discrete_H);
N_W=length(discrete_W);

for i=1
points=[discrete_W]

end