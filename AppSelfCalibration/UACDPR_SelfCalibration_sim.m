% This app allow to simulate the initial pose estimantion of an UACDPR with
% a selfcalibration procedure

clear
close all

% load config file
load('UACDPR_LAB3.mat');
opts = Utilities;
s.DependencyVect=[1,1,1,0,0,1];
MyUACDPR=UACDPR(s);
MyUACDPR= SetOrientType(MyUACDPR,'TaitBryan');
disturb=zeros(6,1);

% direct dynamics
dt = 0.001;
Tmax = 10;
state_guess = zeros(4,1);
sol = HuenDiscreteSolver(@(t,x)DynamicsJointSpace(MyUACDPR,t,x),0:dt:Tmax,state_guess);
x = sol.y;