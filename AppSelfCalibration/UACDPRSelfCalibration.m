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
load('..\UACDPR_SelfCalibration\FreeMotion02_exp.mat');

% compute equilibrium pose
tau = st_out.tensions(:,1);
zita_eq_guess = [0;0;1;0;0;0];
fs_opts = opts.FsolveEqPoses;
zita_eq = fsolve(@(zita) Static(zita,MyUACDPR,disturb, tau),zita_eq_guess,fs_opts);

% integration for guess computation
flag_integration = 0;
if flag_integration
    dt = st_out.t(2)-st_out.t(1);
    Tmax=length(st_out.tensions)*dt; 
    sol = HuenDiscreteSolver(@(t,x) classic_dynamics_log(MyUACDPR, t, x, st_out.tensions, dt), dt:dt:Tmax, [zita_eq; zeros(6,1)]);
    x = sol.y(1:6,:);
else
    x = diag(zita_eq)*ones(6,length(st_out.tensions));
end

% extract data at vicon frequency
X = x(:,st_out.vicon_idx);
cable_length = st_out.cable_length(:,st_out.vicon_idx);
swivel = st_out.swivel(:,st_out.vicon_idx);
epsilon = st_out.epsilon(:,st_out.vicon_idx);
% reduce the number of samplings
X = X(:,1:10:length(X));
cable_length = cable_length(:,1:10:length(cable_length));
swivel = swivel(:,1:10:length(swivel));
epsilon = epsilon(:,1:10:length(epsilon));

% set zero delta_l and delta_sigma
delta_swivel = swivel-swivel(:,1);
delta_length = cable_length-cable_length(:,1);

% selfcalibration optimization
ZV_guess = reshape(X,[6*length(X) 1]);
opts = optimoptions('fminunc','Display', 'iter-detailed',...
'FunctionTolerance',1e-2,'MaxFunctionEvaluation',1e12,'SpecifyObjectiveGradient',true,'CheckGradient',false,... 
'MaxIterations',1000,'OptimalityTolerance',1e-3,'StepTolerance',1e-6,'UseParallel',true);
ZV = fminunc(@(ZV) WLS(ZV, MyUACDPR, delta_swivel, delta_length, epsilon),ZV_guess,opts);
% opts = optimoptions('fminunc','Display', 'iter-detailed',...
% 'FunctionTolerance',1e-2,'MaxFunctionEvaluation',1e12,... 
% 'MaxIterations',200,'OptimalityTolerance',1e-3,'StepTolerance',1e-6,'UseParallel',true);
% ZV = fminunc(@(ZV) WLS_no_grad(ZV, MyUACDPR, delta_swivel, delta_length, epsilon),ZV_guess,opts);
Z = reshape(ZV,[6 length(X)]);

% error result
e_pos = norm(Z(1:3,1)*1000-st_out.pose_v(1:3,1));   % [mm]
e_rot = norm(Z(4:6,1)-st_out.pose_v(4:6,1));        % [deg]

% guess video
% FF=FilmDrawRobotPippo(x,MyUACDPR);