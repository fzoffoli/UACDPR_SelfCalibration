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
pose_bounds = [-0.4 0.4; -0.4 0.4; 0.4 1.4; -pi/2 pi/2; -pi/2 pi/2; -pi/2 pi/2];
k = 10;     % number of measurements (comprising the initial pose)
Z_bounds = repmat(pose_bounds,k,1);

% assign control disturb values
control_disturb.position_bias = 0;                                      %[m]
control_disturb.orientation_bias = 0;                                   %[rad]
control_disturb.position_noise = 0;                                     %[m]
control_disturb.orientation_noise = 0;                                  %[rad]
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
[Z_ideal, tau_ideal,k] = GenerateConfigPosesBrutal(MyUACDPR,grid_axes,pose_bounds,[20 400]);

out.opt_meas_config = reshape(Z_ideal,[n_d k]);
out.n_config = k;
for i = 1:k
    out.static_tensions(:,i) = CalcInverseStaticsAndGradient(MyUACDPR,out.opt_meas_config(:,i));
end

