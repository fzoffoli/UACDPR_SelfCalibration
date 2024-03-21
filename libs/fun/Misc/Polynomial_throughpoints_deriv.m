function dyq=Polynomial_throughpoints_deriv(x_list,y_list,xq)

% y_list= serie in riga (riga determina la variabile, colonna il sample)


%%%LAGRANGE
dyq=zeros(1,size(y_list,1));
for sz=1:size(y_list,1)
    n=length(x_list);

    dP=zeros(1,n);
    for j=1:n
        dy_part2=zeros(1,n);
        for k1=1:n
            h=1;
            for k=1:n
                if k~=j && k~=k1
                    h=h*(xq-x_list(k))/(x_list(j)-x_list(k));
                end
            end
            if k1~=j
                dy_part2(k1)=1/(x_list(j)-x_list(k1))*h;
            end
        end
        dP(j)=sum(dy_part2)*y_list(sz,j);
    end
    
    dyq(1,sz)=sum(dP);
end
end