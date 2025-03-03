%%% This script computes the optimal measurement pose set for the initial
%%% pose self-calibration of UACDPRs and the control inputs for the winches
%%% to reach (ideally) those poses. The control inputs are stored as csv
%%% files to be read from the cdpr software.

clear
close all

% load config file
load('IRMA4U_Mar25.mat');
% load('UACDPR_LAB3.mat');
opts = Utilities;
s.DependencyVect=[1,1,1,0,0,1];
MyUACDPR=UACDPR(s);
n = double(MyUACDPR.CablesNumber);
P = MyUACDPR.PermutMatrix; 
MyUACDPR= SetOrientType(MyUACDPR,'TaitBryan');
disturb=zeros(6,1);

Z_bounds = [-0.4 0.4; -0.4 0.4; 0.4 1.4; -pi/4 pi/4; -pi/4 pi/4; 0 0];

% Optimal measurement pose set computation
%         Z_ideal = ga(@(Z)FitnessFunLengthSwivelAHRS(cdpr_variables,cdpr_parameters,Z,k,method),...
%             k*cdpr_parameters.pose_dim,[],[],[],[],Z_bounds(:,1),Z_bounds(:,2),...
%             @(Z)NonlconWorkspaceBelonging(cdpr_variables,cdpr_parameters,Z,k,ws_info),opts_ga);