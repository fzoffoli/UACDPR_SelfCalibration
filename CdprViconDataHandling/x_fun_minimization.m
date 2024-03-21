function f = x_fun_minimization(x,P)

F = [(P(:,1)-P(:,8))';
     (P(:,4)-P(:,1))';
     (P(:,5)-P(:,4))';
     (P(:,2)-P(:,7))';
     (P(:,3)-P(:,2))';
     (P(:,6)-P(:,3))']*x;

f = F'*F;

end

