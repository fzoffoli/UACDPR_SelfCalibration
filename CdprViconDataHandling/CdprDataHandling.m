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
filename = "..\UACDPR_SelfCalibration\FreeDrive60_4p";

% parsing files
% log_data = parseCableRobotLogFile(strcat(filename,'.log'));
% save('log_data',"log_data");
load("log_data.mat");

%% robot's data processing
show = 1;           % flag to show data
cut_perc = 0.15;    % initial cutting percentage
n_cables = 4;       % insert number of cables
[t_r,epsilon_r,cable_length_r, swivel_r, tensions_r, eps_idx, yaw_home, length_home] = CdprLogProcessing(log_data, n_cables, cut_perc, show);
length_home_json = [2.27617431794265; 2.27617431794265; 2.27617431794265; 2.27617431794265];        %[m] initial lengths json
swivel_home_json = [0.567301627306439; 0.567301627306439; 0.567301627306439; 0.567301627306439];    %[rad]
length_initial_offset = length_home_json - length_home';

%% change inclinometer reference frame
load("RotMatCalib03.mat")   % load structure from calibration
epsilon_r_filt = epsilon_r(:,eps_idx)*pi/180;
epsilon_rv = zeros(size(epsilon_r_filt));
R_z0 = Rz(yaw_home*pi/180);
for i=1:length(epsilon_r_filt)
    R_i = R_z0'*RotMatrixRPY(epsilon_r_filt(:,i));
    epsilon_rv(:,i) = Angles_RPY(R_c*R_i*R_vi); 
end

%% cutting vectors
t_rv = linspace(t_r(eps_idx(1)),t_r(eps_idx(end)),length(epsilon_rv));
t_r_cut = t_r(eps_idx(1):eps_idx(end));
cable_length_r_cut = cable_length_r(:,eps_idx(1):eps_idx(end));
swivel_r_cut = swivel_r(:,eps_idx(1):eps_idx(end));
tensions_r_cut = tensions_r(:,eps_idx(1):eps_idx(end));
epsilon_rv_cut = spline(t_rv,epsilon_rv,t_r_cut);

% final data saving
st.t = t_r_cut;
st.epsilon = epsilon_rv_cut;
st.cable_length = cable_length_r_cut;
st.swivel = swivel_r_cut;
st.tensions = tensions_r_cut;
st.length_home = length_home;
st.length_initial_offset = length_initial_offset;
name=strcat(filename,'_parsed');
save(strcat(name,'.mat'),'st');