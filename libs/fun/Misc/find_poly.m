function A = find_poly(x_pos_list,pos_list,x_vel_list,vel_list,x_acc_list,acc_list)
%the function works with only positional condition or positional and
%velocity condition. If only velocity and/or acceleratione are to be given
%give positional reference as empty inputs. The same goes if only
%positional and acceleration conditions are to be given (i.e. velocity
%conditions are to be set as empty inputs)

%Find the coefficient of the polynomial through specified points.
%coefficients are from lowest to highest exponent (i.e. A(i)*x^(i-1))
if nargin<=2
   x_vel_list=[];
   vel_list=[];
   x_acc_list=[];
   acc_list=[];
elseif nargin<=4
    x_acc_list=[];
   acc_list=[];
end
Q=zeros(length(x_pos_list)+length(x_vel_list)+length(x_acc_list));

for i=1:length(x_pos_list)
    for j=1:length(Q)
    Q(i,j)=x_pos_list(i)^(j-1);
    end
end

for i=1:length(x_vel_list)
    for j=2:length(Q)
    Q(i+length(x_pos_list),j)=(j-1)*x_vel_list(i)^(j-2);
    end
end

for i=1:length(x_acc_list)
    for j=3:length(Q)
    Q(i+length(x_pos_list)+length(x_vel_list),j)=(j-1)*(j-2)*x_acc_list(i)^(j-3);
    end
end

A=Q\[pos_list,vel_list,acc_list].';
end