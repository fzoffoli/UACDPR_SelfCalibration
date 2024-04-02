%%% This script is an adaptation of Coccia's script on Initial Pose
%%% Estimation for UACDPRs

clear
close all

% load config file
load('UACDPR_LAB3.mat');
opts = Utilities;
s.DependencyVect=[1,1,1,0,0,1];
MyUACDPR=UACDPR(s);
n = double(MyUACDPR.CablesNumber);
P = MyUACDPR.PermutMatrix; 
MyUACDPR= SetOrientType(MyUACDPR,'TaitBryan');
disturb=zeros(6,1);

% load experimental data
load('..\UACDPR_SelfCalibration\FreeDrive60_continuous_motion2_parsed.mat');
% compute equilibrium pose
tau = st.tensions(:,1);
zita_eq_guess = [0;0;1;0;0;0];
fs_opts = opts.FsolveEqPoses;
zita_eq = fsolve(@(zita) Static(zita,MyUACDPR,disturb, tau),zita_eq_guess,fs_opts);

% real pose estimation
length_real = st.cable_length + st.length_initial_offset;
pose_real = ComputePoseEstimationLengthsInclinometer(length_real(:,1),st.epsilon(:,1),zita_eq,MyUACDPR);
% save("pose_real.mat",'pose_real');
% load('pose_real.mat');

% errors introduction 
% norm_p = 0.05;
% err_x = 0.30/sqrt(3);
% err_y = err_x;
% err_z = err_x;
% err_x = (2*rand -1) * norm_p;
% err_y = (2*rand -1) * sqrt(norm_p^2-err_x^2);
% err_z = sqrt(norm_p^2-err_x^2-err_y^2);
% err_p = [err_x err_y err_z]';

% err orientation
% norm_ep = deg2rad(2);
% err_roll = (2*pi/180)/sqrt(3);
% err_pitch = err_roll;
% err_yaw = err_roll;
% err_roll = (2*rand -1) * norm_ep;
% err_pitch = (2*rand -1) * sqrt(norm_ep^2-err_roll^2);
% err_yaw = sqrt(norm_ep^2-err_roll^2-err_pitch^2);
% err_ep = [err_roll err_pitch err_yaw]';

pos_errors = GenerateSphericalPoints();

% pose estimation
pose_with_errors = pose_real(:,1) + [err_p; err_ep];
MyUACDPR_temp = SetPoseAndUpdate0KIN(MyUACDPR, pose_with_errors);
length_real_err(:,1) = MyUACDPR_temp.CableLengths_;
delta_length = st.cable_length-st.cable_length(:,1);  
lengths_real_err = delta_length + length_real_err;
pose_est = ComputePoseEstimationLengthsInclinometer(lengths_real_err,st.epsilon,zita_eq,MyUACDPR);

% integration for guess computation
flag_integration = 0;
if flag_integration
    dt = st.t(2)-st.t(1);
    Tmax=length(st.tensions)*dt; 
    sol = HuenDiscreteSolver(@(t,x) classic_dynamics_log(MyUACDPR, t, x, st.tensions, dt), dt:dt:Tmax, [zita_eq; zeros(6,1)]);
    x = sol.y(1:6,:);
else
    x = pose_est;
end

% % extract data at vicon frequency
% X = x(:,st_out.vicon_idx);
% cable_length = st_out.cable_length(:,st_out.vicon_idx);
% swivel = st_out.swivel(:,st_out.vicon_idx);
% epsilon = st_out.epsilon(:,st_out.vicon_idx);

% reduce the number of samplings
N = 200;
X = x(:,1:N:end);
swivel = st.swivel(:,1:N:end);
epsilon = st.epsilon(:,1:N:end);

% set zero delta_l and delta_sigma
delta_swivel = swivel-swivel(:,1);
delta_length = delta_length(:,1:N:end);

tic
% selfcalibration optimization
ZV_guess = reshape(X,[6*length(X) 1]);
opts = optimoptions('fminunc','Display', 'iter-detailed',...
'FunctionTolerance',1e-2,'MaxFunctionEvaluation',1e12,'SpecifyObjectiveGradient',true,'CheckGradient',false,... 
'MaxIterations',1000,'OptimalityTolerance',1e-3,'StepTolerance',1e-6,'UseParallel',true);
ZV = fminunc(@(ZV) WLS(ZV, MyUACDPR, delta_swivel, delta_length, epsilon),ZV_guess,opts);
Z = reshape(ZV,[6 length(X)]);
save("Z",'Z');
toc
%% initial length solution
load("Z.mat");
MyUACDPR_temp = SetPoseAndUpdate0KIN(MyUACDPR,Z(:,1));
length_real_est = MyUACDPR_temp.CableLengths_;
sigma_real_est = zeros(4,1);
for j = 1:4
	sigma_real_est(j,1) = MyUACDPR_temp.Trasmission.Pulley{j}.SwivelAngle;
end

% errors
% err_length = length_real_est - length_real_meas(1);
err_pos = norm(Z(1:3,1)-pose_real(1:3,1))*1000
err_rot = rad2deg(norm(Z(4:6,1)-pose_real(4:6,1)))

% guess video
% FilmDrawRobotPippo(x,MyUACDPR);