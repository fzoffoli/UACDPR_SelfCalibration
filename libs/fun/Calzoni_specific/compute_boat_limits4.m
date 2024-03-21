function points=compute_boat_limits4(L,W,H,PtoG,discret)


discrete_L=-L/2;
discrete_H=-H/2:discret:H/2;
discrete_W=-W/2:discret:W/2;

N_L=length(discrete_L);
N_H=length(discrete_H);
N_W=length(discrete_W);
points=zeros(3,N_L*N_H*N_W);
for i=1:N_L
    for j=1:N_W
        for k=1:N_H
            points(:,k+(j-1)*N_H+(i-1)*(N_W)*N_H)=[discrete_L(i)+discret/2*(-1)^i;discrete_W(j);discrete_H(k)];
        end
    end
end
end