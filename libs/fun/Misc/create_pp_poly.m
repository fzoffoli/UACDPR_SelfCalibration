function [coeff] = create_pp_poly(t0,cut1,cut2,t_max,pos_start,pos_dt_start,pos_ddt_start,error,x_query)
%%%from a to b poly4, from b to c poly1, from c to d poly3
%%%mostly unused, it should output any query point instead of the
%%%coefficient of the first polynomial
a=0;
b=(cut1-t0)/(t_max-t0);
c=(cut2-t0)/(t_max-t0);
d=1;


x_pos{1}=t0+[a,b]*(t_max-t0);
x_vel{1}=t0+[a,b]*(t_max-t0);
x_acc{1}=t0+[a]*(t_max-t0);
% x_vel{1}=t0+[b]*(t_max-t0);
% x_acc{1}=zeros(0);


x_pos{2}=t0+[b,c]*(t_max-t0);
x_vel{2}=zeros(0);
x_acc{2}=zeros(0);

x_pos{3}=t0+[c,d]*(t_max-t0);
x_vel{3}=t0+[c,d]*(t_max-t0);
x_acc{3}=zeros(0);

x_pos_list=t0+[a,b]*(t_max-t0);
x_vel_list=t0+[a,b]*(t_max-t0);
x_acc_list=t0+[a]*(t_max-t0);

x_pos2=t0+[b,c]*(t_max-t0);

x_pos3=t0+[c,d]*(t_max-t0);
x_vel3=t0+[c,d]*(t_max-t0);
% pos_list_x=pos_start(1)+[0,error(1)/4,error(1)];
% pos_list_y=pos_start(2)+[0,error(2)/4,error(2)];
% pos_list_z=pos_start(3)+[0,error(3)/4,error(3)];
% for i=1:3
%     for j=1:3
%     list(i).pos=zeros(3,length(x_pos{i}));
%     list(i).vel=zeros(3,length(x_vel{i}));
%     list(i).acc=zeros(3,length(x_acc{i}));
%     end
% end
for i=1:3 %%%%%%%la struttura è così indicizzata: list(idx_polynom).pos/vel/acc{idx_coord};
    
list(1).pos{i}=pos_start(i)+[0,error(i)*b];
list(2).pos{i}=pos_start(i)+[error(i)*b,error(i)*c];
list(3).pos{i}=pos_start(i)+[error(i)*c,error(i)];

list(1).vel{i}=[pos_dt_start(i),diff([error(i)*b,error(i)*c])/(c-b)/(t_max-t0)];
% list(1).vel{i}=[diff([error(i)*b,error(i)*c])/(c-b)];
list(2).vel{i}=[];
list(3).vel{i}=[diff([error(i)*b,error(i)*c])/(c-b)/(t_max-t0),0];

list(1).acc{i}=pos_ddt_start(i);
% list(1).acc{i}=[];
list(2).acc{i}=[];
list(3).acc{i}=[];

end
for poly=1:3
    coeff{poly,1}=find_poly(x_pos{poly},list(poly).pos{1},x_vel{poly},list(poly).vel{1},x_acc{poly},list(poly).acc{1});
    coeff{poly,2}=find_poly(x_pos{poly},list(poly).pos{2},x_vel{poly},list(poly).vel{2},x_acc{poly},list(poly).acc{2});
    coeff{poly,3}=find_poly(x_pos{poly},list(poly).pos{3},x_vel{poly},list(poly).vel{3},x_acc{poly},list(poly).acc{3});
end
% coeffs=[coeff.x(1,:).',coeff.y(1,:).',coeff.z(1,:).'];
