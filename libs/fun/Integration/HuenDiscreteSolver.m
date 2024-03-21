function sol = HuenDiscreteSolver(f,t,p0)
global flag_save
global flag_break
flag_save=0;

n = length(t);
dim = length(p0);

h = t(2)-t(1);

sol.y = zeros(dim,n);
sol.f = zeros(1,n);

sol.t = t;
sol.y(:,1) = p0;
app = f(t(1),p0);
% sol.f(1:dim/2,1) = app(dim/2+1:dim,1);
sol.f(1:dim,1) = app(1:dim,1);


for i=2:n
    
    y_tilde = sol.y(:,i-1) + h.*sol.f(:,i-1);
%     y_tilde = sol.y(:,i-1) + h.*sol.f(:,i-1);
%      y_tilde = sol.y(:,i-1) + h.*f(t(i-1),sol.y(:,i-1));
%     test= h.*[sol.y(end/2+1:end,i-1);sol.f(:,i-1)]-h.*f(t(i-1),sol.y(:,i-1))

    sol.y(:,i) = sol.y(:,i-1) + h./2.*([sol.f(:,i-1)]...
        +f(t(i),y_tilde));
%         sol.y(:,i) = sol.y(:,i-1) + h./2.*((sol.f(:,i-1))+f(t(i),y_tilde));

        flag_save=1;
    app = f(t(i),sol.y(:,i));
    sol.f(:,i) = app;
%     sol.f(1:dim,i) = app(1:dim,1);
    flag_save=0;
    if flag_break==1
        break
    end
    sol.t_crop(i)=sol.t(i);
end
flag_break=0;
end