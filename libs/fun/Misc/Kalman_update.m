function [x_upd,covar_upd]=Kalman_update(x_pred,covar_pred,C,x_meas,covar_meas)


covar_meas2=[covar_meas];% zeros(3); zeros(3) covar_meas/.01];
 %%K=covar_pred*C.'*inv(C*covar_pred*C.'+covar_meas);  
K=covar_pred*C.'*diag(1./diag(C*covar_pred*C.'+covar_meas2));


x_upd=x_pred+K*(x_meas-C*x_pred); %% (1-k)

covar_upd=covar_pred-K*C*covar_pred;


end