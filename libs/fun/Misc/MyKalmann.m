function [x,P_err]=MyKalmann(x_prev,sigma_prev,A,x2,sigma2)
 
P=A*sigma_prev*A.';

C=[eye(3) zeros(3)];

K=P*C.'*inv(C*P*C.'+diag(sigma2));
if any(isnan(K))
    x=x_prev;
    P_err=P;
% elseif any(isinf(K))
%     x=x2;
%     P_err=sigma2;
else
x=x_prev+K*(x2-C*x_prev);

P_err=P-K*C*P;
end

end