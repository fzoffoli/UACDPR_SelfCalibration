function [err_p,covar_pred]=Kalman(t,t0,dt,err_dard,err_p_gl,contr2,err_std,temp3,covar_gl)
global counter_dard
if t>=t0+4
    %%% Prima dei 4 secondi non sono in tracking; il sistema non evolve con
    %%% la legge usata.
    s_temp=err_p_gl.vel(:,int64(t/dt))+contr2.Gamma*err_p_gl.pos(:,int64(t/dt));  %%% sliding surface
    A=FindErrDerivative(contr2,s_temp); %%% computo A di dx=A*x
    Q=.005*eye(6); %% assegno errore di modello
    x_prev=[err_p_gl.pos(:,int64(t/dt));err_p_gl.vel(:,int64(t/dt))]; %%prendo stato precedente
    [x_pred,covar_pred]=Kalman_predict(x_prev,covar_gl(:,:,int64(t/dt)),A,Q,dt); %%% previsione stima

    if t-counter_dard>=.1 %%% Ogni 100 ms
            counter_dard=t; %% reset counter
%     if counter_dard >=200 %%% Ogni 100 ms
        covar_meas=diag(err_std.^2);
%         x_meas=err_dard;
        x_meas=[err_dard;(err_dard-temp3(1:3,int64(t/dt)-99))/0.1];
%         C=[eye(3), zeros(3)];
        C=eye(6);

        [x_upd,covar_upd]=Kalman_update(x_pred,covar_pred,C,x_meas,covar_meas);
        covar_pred=covar_upd;
        err_p.pos=x_upd(1:3);
        err_p.vel=x_upd(4:6);
%         counter_dard=counter_dard+1;
%         if counter_dard >201
%         counter_dard=0;
%         end
    else
        err_p.pos=x_pred(1:3);
        err_p.vel=x_pred(4:6);
%         counter_dard=counter_dard+1;
    end
else
    %%% Nei primi 4 secondi di simulazione uso l'errore di dardari non
    %%% filtrato per la posizione e differenze finite in velocità
    
%     err_p.pos=x(7:9);
%     err_p.vel=x(16:18);
    err_p.pos=err_dard(1:3);
    vel_raw=diff([err_p_gl.pos(1:3,int64(t/dt)),err_p.pos],1,2)/dt;
    err_p.vel=vel_raw;
%     err_p.vel=    MyLowPassFilt(vel_raw,err_p_gl.vel(:,int64(t/dt)),t,dt,10);
    %%%%% DA MIGLIORARE
%     err_p.pos=e_p(1:3);
%     err_p.vel=diff([err(1:3,int64(t/dt)),e_p],1,2)/dt;  
    covar_pred=eye(6).*10^(-1);
end