function sol = MyIntegrationMethod(f,t,p0)
global flag_save flag_newy0
global flag_break

flag_save=0;
n = length(t);
dim = length(p0);

h = t(2)-t(1);

sol.y = zeros(dim,n);
sol.f = zeros(1,n);
sol.y0=zeros(1,n);

sol.t = t;
sol.y(:,1) = p0;
[app.f,app.y0] = f(t(1),p0);
sol.f(1:dim,1) = app.f(1:dim,1);
sol.y0=app.y0;

for i=2:n
    
%     y_tilde = sol.y0(:,i-1) + h.*sol.f(:,i-1);
    sol.y(:,i) = sol.y0(:,i-1)  + h.*sol.f(:,i-1);
    [app.f,app.y0] = f(t(i),sol.y(:,i));
    sol.f(1:dim,i) = app.f(1:dim,1);
    sol.y0(:,i)=app.y0;          
    sol.t_crop(i)=sol.t(i);
    
    if flag_break==1
        break
    end
end
flag_break=0;
end