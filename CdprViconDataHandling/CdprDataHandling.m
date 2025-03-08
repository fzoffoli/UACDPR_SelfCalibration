%%% This script allows to extract data from the output .log file from the
%%% cdpr_alars app. The tensions from the loadcells are filtered and data
%%% from the inclinometer is resampled since its min sampling freq is 400Hz

clear
close all

% insert here your filename path, call the two log files with the same name
filename = "sc_8_medium_a";

% parsing files
if isfile("log_data.mat")
    load("log_data.mat")
else
    log_data = parseCableRobotLogFile(strcat(filename,'.log'));
    save('log_data',"log_data");
end

%% robot's data processing
cut_perc = 0.15;    % initial cutting percentage
n_cables = 4;       % insert number of cables
homing_val.lengths = [2.27254824660233; 2.27300821626030; 2.27300821626030; 2.27254824660233];          %[m] initial lengths json
homing_val.swivels = [0.568104732474427; -0.568104732474427; 0.568104732474427; -0.568104732474427];    %[rad]
[t_r,epsilon_r,cable_length_r,swivel_r,tensions_r,target_tensions_r] = CdprLogProcessing(log_data, n_cables, homing_val);

%% change inclinometer reference frame TODO: CORRECT FROM HERE!
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