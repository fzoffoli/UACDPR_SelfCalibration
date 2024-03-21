function points=compute_boat_limits2(L,W,H,PtoG)


discrete_L=[-L/2,L/2];
discrete_H=[-H/2,H/2];
discrete_W=[-W/2,W/2];

N_L=length(discrete_L);
N_H=length(discrete_H);
N_W=length(discrete_W);
for i=1:N_L
    for j=1:N_W
        for k=1:N_H
            points(:,k+(j-1)*2+(i-1)*(2)*2)=[discrete_L(i);discrete_W(j);discrete_H(k)];
        end
    end
end
end