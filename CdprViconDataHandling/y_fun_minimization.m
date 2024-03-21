function f = y_fun_minimization(y,P)

F = [(P(:,8)-P(:,2))';
     (P(:,7)-P(:,1))';
     (P(:,5)-P(:,3))';
     (P(:,6)-P(:,4))']*y;

f = F'*F;

end

