function [t_v_s,pose_v_s] = vicon_data_handling(vicon_data,dt_v,dt_s)

    % DATA PARSING
    indx_v = vicon_data(:,1);
    t_v = zeros(1,length(indx_v));
    for i = 1:length(indx_v)
        t_v(i) = dt_v*(indx_v(i)-1);    
    end
    
    t_v_s = t_v(1):dt_s:t_v(end);
    vicon_data_s = (spline(t_v,vicon_data',t_v_s))';
    
    P = zeros(3,length(vicon_data_s),9); %Hai 3xlunghezza dei dati, e poi x9 perchè hai 9 riscontri sull'EE
    for ii=1:length(vicon_data_s)
        for k = 1:9
            P(:,ii,k) = vicon_data_s(ii,(3*(k-1)+2):(3*(k-1)+4))';
        end
    end

    % DATA FILTERING
    d_filt = designfilt('lowpassfir', ...
    'FilterOrder', 50,'PassBandFrequency', 4,'StopBandFrequency',8,...
    'DesignMethod','equiripple','SampleRate',1/dt_s);   
    for k = 1:9
        % Pk = P(:,:,k);
        for j=1:3            
            Pk_j_mean = mean(P(j,:,k));
            Pk_j_filt = Pk_j_mean + filtfilt(d_filt,P(j,:,k)-Pk_j_mean);
            P_filt(j,:,k) = Pk_j_filt;
%             plot(P(j,:,k));
%             hold on
%             grid on
%             plot(Pk_j_filt);
        end
    end
    % save('P_filt',"P_filt");
    % POINTS TO POSE CONVERSION
    pose_v_s = zeros(6,length(vicon_data_s));
    opts = optimoptions(@fmincon,'UseParallel',false);
    for ii=1:length(vicon_data_s)
        P_filt_ii = permute(P_filt(:,ii,:),[1 3 2]);
%         fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options)
        x = fmincon(@(x) x_fun_minimization(x,P_filt_ii),[1; 0; 0],(P_filt_ii(:,2)-P_filt_ii(:,1))',0,[],[],[],[],[],opts);
        x = x/norm(x);
        
        y = fmincon(@(y) y_fun_minimization(y,P_filt_ii),[0; 1; 0],(P_filt_ii(:,1)-P_filt_ii(:,4))',0,[],[],[],[],[],opts);
        y = y/norm(y);
        
        z = cross(x,y);
        
        R = [x y z];        % R01
        
        pose_v_s(1:3,ii) = P_filt(:,ii,9);
        % Roll pitch yaw angles: RPY parametrization 
%         pose_v_s(4,ii) = atan2(-R(2,3),R(3,3));                      % ROLL
%         pose_v_s(5,ii) = atan2(R(1,3),R(3,3)/cos(pose_v_s(4,ii)));   % PITCH
%         pose_v_s(6,ii) = atan2(-R(1,2),R(1,1));                      % YAW  

        RPY = Angles_RPY(R);
        pose_v_s(4,ii) = RPY(1);                      % ROLL
        pose_v_s(5,ii) = RPY(2);                      % PITCH
        pose_v_s(6,ii) = RPY(3);                      % YAW  
    end 
    

end

