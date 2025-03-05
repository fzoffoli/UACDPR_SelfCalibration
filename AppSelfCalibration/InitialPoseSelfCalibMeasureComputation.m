%%% This script computes the optimal measurement pose set for the initial
%%% pose self-calibration of UACDPRs and the control inputs for the winches
%%% to reach (ideally) those poses. The control inputs are stored as csv
%%% files to be read from the cdpr software.

clear
close all

% load config file
load('IRMA4U_Mar25.mat');
utils = Utilities;
s.DependencyVect=[1,1,1,0,0,1];

% istantiate robot object
MyUACDPR=UACDPR(s); 
MyUACDPR= SetOrientType(MyUACDPR,'TaitBryan');
n_d = length(MyUACDPR.DependencyVect);

% set workspace translational bounds
pose_bounds = [-0.3 0.3; -0.3 0.3; 0.4 1; -pi/2 pi/2; -pi/2 pi/2; -pi/2 pi/2];
% k = 10;     % number of measurements (comprising the initial pose)
% Z_bounds = repmat(pose_bounds,k,1);

% assign control disturb values
control_disturb.position_bias = 0.04;                                   %[m]
control_disturb.orientation_bias = deg2rad(2);                          %[rad]
control_disturb.position_noise = 0.02;                                  %[m]
control_disturb.orientation_noise = deg2rad(1);                         %[rad]
% assign sensor disturb values
sensor_disturb.swivel_noise = deg2rad(1);                               %[rad]
sensor_disturb.length_noise = 0.01;                                     %[m]
sensor_disturb.AHRS_noise = deg2rad(2);                                 %[rad]
sensor_disturb.loadcell_noise = 10;                                     %[N]

% optimal measurement pose set computation
% opts_ga = optimoptions('ga','UseParallel',true,'Display','iter');
% Z_ideal = ga(@(Z)FitnessFunLoadcellLengthSwivelAHRS(MyUACDPR,Z,k,sensor_disturb),...
%     k*n_d,[],[],[],[],Z_bounds(:,1),Z_bounds(:,2),...
%     @(Z)NonlconWorkspaceBelonging(MyUACDPR,Z,k,[20 400]),opts_ga);
% Z_ideal = ga(@(Z)FitnessFunLoadcellLengthSwivelAHRS(MyUACDPR,Z,k,sensor_disturb),...
%     k*n_d,[],[],[],[],Z_bounds(:,1),Z_bounds(:,2),...
%     [],opts_ga);

% measurement pose set computation
grid_axes = [2 2 2];
[Z_ideal,k] = GenerateConfigPosesBrutal(MyUACDPR,grid_axes,pose_bounds,[20 400]);

out.opt_meas_config = reshape(Z_ideal,[n_d k]);
out.n_config = k;
for i = 1:k
    out.static_tensions(:,i) = CalcInverseStaticsAndGradient(MyUACDPR,out.opt_meas_config(:,i));
end
save('sc_control_target.mat',"out");

% IK simulation
Z_ideal = reshape(Z_ideal,[n_d*k 1]);
[X_real, loadcell_meas, delta_length_meas, delta_swivel_meas, delta_yaw_meas, roll_meas, pitch_meas] = ...
        ControlSimLengthLoadcellSwivelAHRS(MyUACDPR,Z_ideal,k,control_disturb,sensor_disturb);

% guess generation
X_guess = Z_ideal;

% solve self-calibration problem
opts = optimoptions('fmincon','FunctionTolerance',1e-10,'OptimalityTolerance',1e-8, ...
    'StepTolerance',1e-10,'UseParallel',true,'MaxFunctionEvaluations',1e+6,'MaxIterations',1e+5);
tic
X_sol = fmincon(@(X)CostFunLoadcellLengthSwivelAHRS(MyUACDPR,X,k-1,loadcell_meas,delta_length_meas,...
        delta_swivel_meas,roll_meas,pitch_meas,delta_yaw_meas,sensor_disturb),X_guess,[],[],[],[],[],[],[],opts);
self_calib_comp_time=toc;

InitialPositionErrorNorm = norm(X_real(1:3)-X_sol(1:3));
MyUACDPR=SetPoseAndUpdate0KIN(MyUACDPR,X_real(1:6));
angle_init_real = acos((MyUACDPR.EndEffector.RotMatrix(1,1)+MyUACDPR.EndEffector.RotMatrix(2,2)+MyUACDPR.EndEffector.RotMatrix(3,3)-1)/2);
MyUACDPR=SetPoseAndUpdate0KIN(MyUACDPR,X_sol(1:6));
angle_init_sol = acos((MyUACDPR.EndEffector.RotMatrix(1,1)+MyUACDPR.EndEffector.RotMatrix(2,2)+MyUACDPR.EndEffector.RotMatrix(3,3)-1)/2);
InitialOrientationError = rad2deg(abs(angle_init_sol-angle_init_real));