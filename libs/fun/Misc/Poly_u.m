function [coeff,u,u_dt,u_ddt] = Poly_u(Tmax,dt,k)


time = (0:dt:Tmax);
n = length(time);
X=[0 0 0 0 0 0 0 0 0 0];
for i=1:length(k)
    X(i)=k(i);
end
k1=X(1);
k2=X(2);
k3=X(3);
k4=X(4);
k5=X(5);
k6=X(6);
k7=X(7);
k8=X(8);
k9=X(9);
k10=X(10);

    alfa = (1-k1*Tmax^2-k2*Tmax^3-k3*Tmax^4-k4*Tmax^5-k5*Tmax^6-k6*Tmax^7-k7*Tmax^8-k8*Tmax^9-k9*Tmax^10-k10*Tmax^11)/Tmax;
    
    gamma = alfa*time+k1*time.^2+k2*time.^3+k3*time.^4+k4*time.^5+k5*time.^6+k6*time.^7+k7*time.^8+k8*time.^9+k9*time.^10+k10*time.^11;
    
    gamma_dt = alfa+2*k1*time+3*k2*time.^2+4*k3*time.^3+5*k4*time.^4+6*k5*time.^5+7*k6*time.^6+8*k7*time.^7+9*k8*time.^8+10*k9*time.^9+11*k10*time.^10;
    gamma_ddt = 2*k1 + 3*2*k2*time+4*3*k3*time.^2+5*4*k4*time.^3+6*5*k5*time.^4+7*6*k6*time.^5+8*7*k7*time.^6+9*8*k8*time.^7+10*9*k9*time.^8+11*10*k10*time.^9;

    u = 0;
    u_dgamma = 0;
    u_ddgamma = 0;
    
    r=3;

    for i=(r+1):(2*r+1)
        at=((-1)^(i-r-1)*factorial(2*r+1))/(i*factorial(r)*factorial(i-r-1)*factorial(2*r+1-i));
        u = u+at*(gamma).^i;
        coeff(i+1)=at*gamma(1/dt+1)^i;
        u_dgamma = i*at*(gamma).^(i-1)+u_dgamma;
        u_ddgamma = (i-1)*i*at*(gamma).^(i-2)+ u_ddgamma;
    end

%     
%     for i=(r+1):(2*r+1)
%         at=((-1)^(i-r-1)*factorial(2*r+1))/(i*factorial(r)*factorial(i-r-1)*factorial(2*r+1-i));
%         u = u+at*(gamma).^i;
%         u_dgamma = i*at*(gamma).^(i-1)+u_dgamma;
%         u_ddgamma = (i-1)*i*at*(gamma).^(i-2)+ u_ddgamma;
%     end
%     
    u_dt=gamma_dt.*u_dgamma;
    u_ddt = gamma_dt.^2.*u_ddgamma + gamma_ddt.*u_dgamma;
end

