%%% This script allows to extract data from the output .log file from the
%%% cdpr_alars app. The absolute values of cable lengths and swivel angles
%%% are obrained from an initial logging in homing position.

clear
close all

% insert here your filename path, call the two log files with the same name
filename = "sc_27_final";

% parsing files
log_data = parseCableRobotLogFile(strcat(filename,'.log'));

% robot's data processing
n_cables = 4;
homing_val.lengths = [2.27254824660233; 2.27300821626030; 2.27300821626030; 2.27254824660233];          %[m] initial lengths json
homing_val.swivels = [0.568104732474427; -0.568104732474427; 0.568104732474427; -0.568104732474427];    %[rad] initial angles json
[t_r,epsilon_r,cable_length_r,swivel_r,tensions_r,target_tensions_r] = CdprLogProcessing(log_data, n_cables, homing_val);

% final data saving
st.t = t_r;
st.epsilon = epsilon_r;
st.cable_length = cable_length_r;
st.swivel = swivel_r;
st.tensions = tensions_r;
st.target_tensions = target_tensions_r;
name=strcat(filename,'_parsed');
save(strcat(name,'.mat'),'st');