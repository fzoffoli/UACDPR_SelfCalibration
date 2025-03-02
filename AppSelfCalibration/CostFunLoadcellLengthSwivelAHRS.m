function [fun,grad] = CostFunLoadcellLengthSwivelAHRS(MyUACDPR, X, tau, delta_length, delta_swivel, roll, pitch, delta_yaw, sensor_disturb)

%% old version
% m = int64((length(zv)/6));
% z = zeros(6,m);
% for i = 1:m
%     z(:,i) = zv(((i-1)*6+1):(i*6));
% end
% % Ottimizzazione
% 
% %errori
% % e_sigma = deg2rad(3); %rad
% % e_l = 0.01; %m
% % e_r_p = deg2rad(0.5); %rad
% % e_y = deg2rad(3); %rad
% 
% % e_sigma = 1; %rad
% % e_l = 1; %m
% % e_r_p = 1; %rad
% % e_y = 1; %rad
% 
% e_sigma = deg2rad(1); %rad
% e_l = 0.005; %m
% e_r_p = deg2rad(0.5); %rad
% e_y = deg2rad(3); %rad
% 
% % sigma_eq, length_eq
% temp = SetPoseAndUpdate0KIN(MyUACDPR,z(:,1));
% length_eq = temp.CableLengths_;
% swivel_eq = zeros(4,1);
% for j = 1:4
%     swivel_eq(j,1) = temp.Trasmission.Pulley{j}.SwivelAngle;
% end
% 
% %swivel, lengths, rpy
% for i = 1:m-1
%     temp = SetPoseAndUpdate0KIN(MyUACDPR,z(:,i+1));
%     lengths(:,i) = temp.CableLengths_;
%     for j = 1:4
%         swivel(j,i) = temp.Trasmission.Pulley{j}.SwivelAngle;
%     end
% end
% rpy = z(4:6, :);
% 
% % Costruisco W e F
% F_s = zeros(1,4*(m-1));
% F_l = zeros(1,4*(m-1));
% F_rpy = zeros(1,3*m);
% d2 = zeros(1,3*m);
% d = [ones(4*(m-1),1)*1/(e_sigma^2);ones(4*(m-1),1)*1/(e_l^2)];
% for i = 1:m-1
%     F_s(i*4-3:i*4) = swivel(:,i) - swivel_eq - delta_s(:,i);
%     F_l(i*4-3:i*4) = lengths(:,i) - length_eq - delta_l(:,i);
% end
% for i =1:m
%     d2(i*3-2:i*3) = [1/(e_r_p^2) 1/(e_r_p^2) 1/(e_y^2)];
%     F_rpy(i*3-2:i*3) = rpy(:,i) - delta_rpy(:,i);
% end
% w = [d;d2'];
% W = diag(w);
% F = [F_s F_l F_rpy]';
% 
% % Costruisco J
% 
% % J_swivel_diag = zeros(4*(m-1),6*(m-1));
% J_length_diag = zeros(4*(m-1),6*(m-1));
% J_rpy = [];
% 
% temp = SetPoseAndUpdate0KIN(MyUACDPR,z(:,1));
% J_swivel_0 = zeros(4*(m-1),6);
% J_length_0 = zeros(4*(m-1),6);
% J_swivel_diag = zeros(4*(m-1),6*(m-1));
% 
% % J_swivel_0 e J_length_0
% for i = 1:m-1
%     J_swivel_0 ((i-1)*4+1:i*4,:) = - temp.AnalJac.Swivel;
%     J_length_0 ((i-1)*4+1:i*4,:) = - temp.AnalJac.Cables;
% end
% 
% % J_swivel_diag e J_length_diag
% for i = 1:m-1
%     temp = SetPoseAndUpdate0KIN(MyUACDPR,z(:,i+1));
%     J_swivel_1m = temp.AnalJac.Swivel;
%     J_length_1m = temp.AnalJac.Cables;
%     J_swivel_diag((i-1)*4+1:i*4,(i-1)*6+1:i*6) = J_swivel_1m;
%     J_length_diag((i-1)*4+1:i*4,(i-1)*6+1:i*6) = J_length_1m;
% end
% 
% % J_rpy
% for i = 1:m
%     J_rpy_1m = [zeros(3,3) eye(3,3)];
%     J_rpy((i-1)*3+1:i*3,(i-1)*6+1:i*6) = J_rpy_1m;
% end
% 
% % J_swivel e J_length
% J_swivel = [J_swivel_0 J_swivel_diag];
% J_length = [J_length_0 J_length_diag];
% 
% % Fun e grad
% J = [J_swivel; J_length; J_rpy];
% 
% % Minimizzazione scalare fun
% fun = 0.5*(F'*W*F);
% grad = J'*W*F;
% 
% % Minimizzazione vettore F
% % fun = F;
% % grad = J;
% 
% % J = [nx6];
% % grad = J'WF;

%% This is the cost function for a self-calibration optimization problem
% where the initial pose of a CDPR has to be estimated using loadcells 
% measures, swivel angle variation measures and euler angles from an AHRS.
% Read Zoffoli2025 Robotica for the complete formulation.

k = length(X)/length(MyUACDPR.DependencyVect);

% set constants for the weighing matrix
force_max = sensor_disturb.loadcell_noise;
length_max = sensor_disturb.length_noise;
sigma_max = sensor_disturb.swivel_noise;
epsilon_max = sensor_disturb.AHRS_noise;

% extract variables and parameters 
zeta_0 = X(1:length(MyUACDPR.DependencyVect));
Z = X(length(MyUACDPR.DependencyVect)+1:length(MyUACDPR.DependencyVect)*(k+1));
MyUACDPR = MyUACDPR.SetPoseAndUpdate0KIN(zeta_0);
length_0 = MyUACDPR.Trasmission.CableLengths;
sigma_0 = zeros(MyUACDPR.CablesNumber,1);
for j_=1:MyUACDPR.CablesNumber
    sigma_0(j_) = MyUACDPR.Trasmission.Pulley{1,j_}.SwivelAngle;
end
psi_0 = zeta_0(end);

% fill the residual vector
% n = cdpr_p.n_cables;
% f_tau = zeros(k*6,1);
% f_length = zeros(k*n,1);
% f_sigma = zeros(k*n,1);
% f_epsilon = zeros(k*3,1);
% for i=1:k
%     zeta_k = Z(i*cdpr_p.pose_dim-(cdpr_p.pose_dim-1):i*cdpr_p.pose_dim);
%     cdpr_v = UpdateIKZeroOrd(zeta_k(1:3),zeta_k(4:6),cdpr_p,cdpr_v);
%     cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
%     length_model = zeros(n,1);
%     sigma_model=zeros(n,1);
%     for j_=1:n
%         length_model(j_) = cdpr_v.cable(j_).complete_length;
%         sigma_model(j_) = cdpr_v.cable(j_).swivel_ang;
%     end
%     % [tau_c, tau_d] = CalcTDBarycentric(cdpr_v,cdpr_p,[10 500]);
%     % f_tau(i*n-(n-1):i*n) = [tau_d;tau_c]-tau(:,i); 
%     % wrench = cdpr_v.geometric_jacobian_l*tau(:,i)-cdpr_v.platform.ext_load; 
%     % f_tau(i*6-(6-1):i*6) = [wrench(1:3)./force_max; wrench(4:6)./moment_max];
%     tau_zeta = CalcTDClosedForm(cdpr_v,cdpr_p,[10 500]);
%     f_tau(i*n-(n-1):i*n) = (tau_zeta-tau(:,i)); 
%     f_length(i*n-(n-1):i*n) = length_model-length_0-delta_length(:,i); 
%     f_sigma(i*n-(n-1):i*n) = (sigma_model-sigma_0-delta_sigma(:,i));
%     f_epsilon(i*3-2:i*3) = (zeta_k(4:6)-[roll(i);pitch(i);delta_yaw(i)]-[0;0;psi_0]);
% end
% F = [f_tau; f_length; f_sigma; f_epsilon];
% % W = diag([repmat([1/(force_max^2); 1/(force_max^2); 1/(force_max^2); ...
% %                     1/(moment_max^2); 1/(moment_max^2); 1/(moment_max^2)],[k 1]); ...
% %             ones(size(f_sigma))./(sigma_max^2); ...
% %             ones(size(f_epsilon))./(epsilon_max^2)]);
% W = diag([ones(size(f_tau))./(force_max^2); ...
%             ones(size(f_length))./(length_max^2); ...
%             ones(size(f_sigma))./(sigma_max^2); ...
%             ones(size(f_epsilon))./(epsilon_max^2)]);
% f = 0.5*F'*W*F;
end