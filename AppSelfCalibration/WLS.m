function [fun,grad] = WLS(zv, ROBOT, delta_s, delta_l, delta_rpy)
    
    m = int64((length(zv)/6));
    z = zeros(6,m);
    for i = 1:m
        z(:,i) = zv(((i-1)*6+1):(i*6));
    end
    %% Ottimizzazione

    %errori
    % e_sigma = deg2rad(3); %rad
    % e_l = 0.01; %m
    % e_r_p = deg2rad(0.5); %rad
    % e_y = deg2rad(3); %rad

    % e_sigma = 1; %rad
    % e_l = 1; %m
    % e_r_p = 1; %rad
    % e_y = 1; %rad

    e_sigma = deg2rad(1); %rad
    e_l = 0.005; %m
    e_r_p = deg2rad(0.5); %rad
    e_y = deg2rad(3); %rad
    
    % sigma_eq, length_eq
    temp = SetPoseAndUpdate0KIN(ROBOT,z(:,1));
    length_eq = temp.CableLengths_;
    swivel_eq = zeros(4,1);
    for j = 1:4
	    swivel_eq(j,1) = temp.Trasmission.Pulley{j}.SwivelAngle;
    end

    %swivel, lengths, rpy
    for i = 1:m-1
        temp = SetPoseAndUpdate0KIN(ROBOT,z(:,i+1));   
        lengths(:,i) = temp.CableLengths_;
        for j = 1:4
	        swivel(j,i) = temp.Trasmission.Pulley{j}.SwivelAngle;
        end
    end
    rpy = z(4:6, :);
    
    % Costruisco W e F
    F_s = zeros(1,4*(m-1));
    F_l = zeros(1,4*(m-1));
    F_rpy = zeros(1,3*m);
    d2 = zeros(1,3*m);
    d = [ones(4*(m-1),1)*1/(e_sigma^2);ones(4*(m-1),1)*1/(e_l^2)];
    for i = 1:m-1
        F_s(i*4-3:i*4) = swivel(:,i) - swivel_eq - delta_s(:,i);
        F_l(i*4-3:i*4) = lengths(:,i) - length_eq - delta_l(:,i);
    end
    for i =1:m
        d2(i*3-2:i*3) = [1/(e_r_p^2) 1/(e_r_p^2) 1/(e_y^2)];
        F_rpy(i*3-2:i*3) = rpy(:,i) - delta_rpy(:,i);
    end
    w = [d;d2'];
    W = diag(w);
    F = [F_s F_l F_rpy]';
    
    % Costruisco J
    
    % J_swivel_diag = zeros(4*(m-1),6*(m-1));
    J_length_diag = zeros(4*(m-1),6*(m-1));
    J_rpy = [];

    temp = SetPoseAndUpdate0KIN(ROBOT,z(:,1));
    J_swivel_0 = zeros(4*(m-1),6);
    J_length_0 = zeros(4*(m-1),6);
    J_swivel_diag = zeros(4*(m-1),6*(m-1));
    
    % J_swivel_0 e J_length_0
    for i = 1:m-1
        J_swivel_0 ((i-1)*4+1:i*4,:) = - temp.AnalJac.Swivel;
        J_length_0 ((i-1)*4+1:i*4,:) = - temp.AnalJac.Cables;
    end

    % J_swivel_diag e J_length_diag
    for i = 1:m-1
        temp = SetPoseAndUpdate0KIN(ROBOT,z(:,i+1));   
        J_swivel_1m = temp.AnalJac.Swivel; 
        J_length_1m = temp.AnalJac.Cables; 
        J_swivel_diag((i-1)*4+1:i*4,(i-1)*6+1:i*6) = J_swivel_1m;
        J_length_diag((i-1)*4+1:i*4,(i-1)*6+1:i*6) = J_length_1m;
    end
    
    % J_rpy
    for i = 1:m
        J_rpy_1m = [zeros(3,3) eye(3,3)]; 
        J_rpy((i-1)*3+1:i*3,(i-1)*6+1:i*6) = J_rpy_1m;
    end

    % J_swivel e J_length
    J_swivel = [J_swivel_0 J_swivel_diag];
    J_length = [J_length_0 J_length_diag];

    % Fun e grad
    J = [J_swivel; J_length; J_rpy];

    % Minimizzazione scalare fun
    fun = 0.5*(F'*W*F);
    grad = J'*W*F;

    % Minimizzazione vettore F
    % fun = F;
    % grad = J;

    % J = [nx6];
    % grad = J'WF;
   

end