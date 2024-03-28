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
load(['..\UACDPR_SelfCalibration\FreeDrive60_continuous_motion2_parsed.mat']);

% compute equilibrium pose
tau = st.tensions(:,1);
zita_eq_guess = [0;0;1;0;0;0];
fs_opts = opts.FsolveEqPoses;
zita_eq = fsolve(@(zita) Static(zita,MyUACDPR,disturb, tau),zita_eq_guess,fs_opts);

% pose estimation
pose_est = ComputePoseEstimationLengthsInclinometer(st,zita_eq,MyUACDPR);

% integration for guess computation
flag_integration = 0;
if flag_integration
    dt = st.t(2)-st.t(1);
    Tmax=length(st.tensions)*dt; 
    sol = HuenDiscreteSolver(@(t,x) classic_dynamics_log(MyUACDPR, t, x, st.tensions, dt), dt:dt:Tmax, [zita_eq; zeros(6,1)]);
    x = sol.y(1:6,:);
else
%     x = diag(zita_eq)*ones(6,length(st.tensions));
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
X = X+[rand(3,1)*0.2; rand(3,1)*3*pi/180];
cable_length = st.cable_length(:,1:N:end);
swivel = st.swivel(:,1:N:end);
epsilon = st.epsilon(:,1:N:end); %If correct -> length became end

% set zero delta_l and delta_sigma
delta_swivel = swivel-swivel(:,1);
delta_length = cable_length-cable_length(:,1);

% selfcalibration optimization
ZV_guess = reshape(X,[6*length(X) 1]);
opts = optimoptions('fminunc','Display', 'iter-detailed',...
'FunctionTolerance',1e-2,'MaxFunctionEvaluation',1e12,'SpecifyObjectiveGradient',true,'CheckGradient',false,... 
'MaxIterations',1000,'OptimalityTolerance',1e-3,'StepTolerance',1e-6,'UseParallel',true);
ZV = fminunc(@(ZV) WLS(ZV, MyUACDPR, delta_swivel, delta_length, epsilon),ZV_guess,opts);
Z = reshape(ZV,[6 length(X)]);
save("Z",'Z');

%% initial length solution
load("Z.mat");
temp = SetPoseAndUpdate0KIN(MyUACDPR,Z(:,1));
length_real_est = temp.CableLengths_;
sigma_real_est = zeros(4,1);
for j = 1:4
	sigma_real_est(j,1) = temp.Trasmission.Pulley{j}.SwivelAngle;
end

% errors
% err_length = length_real_est - length_real_meas(1);
err_pos = norm(Z(1:3,1)-pose_est(1:3,1))*1000
err_rot = rad2deg(norm(Z(4:6,1)-pose_est(4:6,1)))

% guess video
% FF=FilmDrawRobotPippo(x,MyUACDPR);