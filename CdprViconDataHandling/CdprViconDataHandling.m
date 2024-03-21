%%% This script allows to extract data from the output .log file from the
%%% cdpr_alars app and from the output .txt file from vicon, in case a
%%% standard allocation of the markers is used to construc the object
%%% platform. The correct procedure for data acquisition is described in
%%% Vicon-Inclinometer_Calibration (GitHub IRMA-LAB). The tensions from the
%%% loadcells are filtered and data from the inclinometer is resampled, due
%%% to problems of real-time communication (update mar 2024).The inclinometer 
%%% angles are then brought to the vicon reference frame, and the temporal 
%%% vector of the robot's data is shifted to compare the result of the two
%%% acquisitions.

clear
close all

% insert here your filename path, call the two log files with the same name
filename = "..\Code\Calibration\Calibration of Real System\FreeMotion02";

% importing .txt file
vicon_data=readmatrix(strcat(filename,'.txt'));
vicon_data(1:3,:)=[];
vicon_data(:,2)=[];

% parsing files
log_data = parseCableRobotLogFile(strcat(filename,'.log')); 
dt_v = 1/100; % insert vicon sampling period [s]
[t_v,pose_v] = vicon_data_handling(vicon_data,dt_v,dt_v);

% robot's data processing
show = 1;           % flag to show data
cut_perc = 0.25;    % initial cutting percentage
n_cables = 4;       % insert number of cables
[t_r,epsilon_r,cable_length_r, swivel_r, tensions_r, eps_idx, yaw_home] = CdprLogProcessing(log_data, n_cables, cut_perc, show);

st.t_r = t_r;
st.epsilon_r = epsilon_r;
st.cable_length_r = cable_length_r;
st.swivel_r = swivel_r;
st.tensions_r = tensions_r;
name=strcat(filename,'_parsed');
save(strcat(name,'.mat'),'st');

% change inclinometer reference frame
load("RotMatCalib03.mat")   % load structure from calibration
epsilon_r_filt = epsilon_r(:,eps_idx)*pi/180;
epsilon_rv = zeros(size(epsilon_r_filt));
R_z0 = Rz(yaw_home*pi/180);
for i=1:length(epsilon_r_filt)
    R_i = R_z0'*RotMatrixRPY(epsilon_r_filt(:,i));
    epsilon_rv(:,i) = Angles_RPY(R_c*R_i*R_vi); 
end

% aligning vectors
st_out = CdprViconCutAndAlign(eps_idx,epsilon_rv,t_r,cable_length_r,swivel_r,tensions_r,pose_v,t_v,show);

% save data structure
name=strcat(filename,'_exp');
save(strcat(name,'.mat'),"st_out");