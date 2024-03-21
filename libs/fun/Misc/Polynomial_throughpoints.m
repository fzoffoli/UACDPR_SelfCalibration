function yq=Polynomial_throughpoints(x_list,y_list,xq)

% y_list= serie in riga (riga determina la variabile, colonna il sample)


%%%LAGRANGE
yq=zeros(1,size(y_list,1));
for sz=1:size(y_list,1)
n=length(x_list);
y_part=zeros(1,n);
for j=1:n
    h=1;
    for k=1:n
        if k~=j
        h=h*(xq-x_list(k))/(x_list(j)-x_list(k));  
        end
    end
    y_part(j)=y_list(sz,j)*h;
end

yq(1,sz)=sum(y_part);
end
end