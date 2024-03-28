% This function filter the data of the parsed .log file from the app
% cdpr_alars
% INPUT parsed log data from file .log
% OUTPUT time vector, RPY vector with erased repetitions, cable lengths
% vector, swivel angles vector, tension vector filtered, index of epsilon
% wrt others vectors, yaw calibration angle.

function [t_r,epsilon_r,cable_length_r, swivel_r, tensions_r, eps_idx, yaw_home, length_home] = CdprLogProcessing(log_data, n_cables, cut_perc, show)

% Inclinometer data extraction
t = log_data.inclinometer_data.timestamp;
epsilon = [log_data.inclinometer_data.values.roll;        
log_data.inclinometer_data.values.pitch;
log_data.inclinometer_data.values.yaw];

% Actuators data extraction
invalid_data = mod(length(log_data.actuator_status.timestamp),n_cables);
Nlswivel = (length(log_data.actuator_status.timestamp)-invalid_data); 
for k = 1:4:Nlswivel
    kk = (k+3)/4;
    for j = 1:n_cables
        cable_length(j,kk) = log_data.actuator_status.values.cable_length(k+j-1);
        % The "-" sign is added because the transmission ratio (swivel encoder angle/count swivel encoder) is wrong: 
        % A MINUS IS NEEDED to measure the ACTUAL SWIVEL ANGLES
        swivel(j,kk) = - log_data.actuator_status.values.pulley_angle(k+j-1);
        tensions(j,kk) = log_data.actuator_status.values.cable_tension(k+j-1);
        target_tensions(j,kk) = log_data.actuator_status.values.target(k+j-1);
    end
end

% Homing yaw evaluation
delta = 0.5;
i = 2;
yaw_temp(1) = epsilon(3,1);
length_temp(:,1) = cable_length(:,1); 
while (epsilon(3,i)-epsilon(3,1)<delta)
    yaw_temp(i) = epsilon(3,i);
    length_temp(:,i) = cable_length(:,i); 
    i=i+1;
end
yaw_home = mean(yaw_temp);
for i = 1:4
    length_home(i) = mean(length_temp(i));
end

% Cutting data with the specified percentage
epsilon_r=epsilon(:,int64(cut_perc*length(epsilon)):end);
swivel_r=swivel(:,int64(cut_perc*length(swivel)):end);
cable_length_r=cable_length(:,int64(cut_perc*length(cable_length)):end);
tensions=tensions(:,int64(cut_perc*length(tensions)):end);
t_r = t(int64(cut_perc*length(t)):end);
if show 
    figure()
    subplot(2,1,1)
    plot(t,epsilon(1,:))
    subplot(2,1,2)
    plot(t_r,epsilon_r(1,:))
    title('Cutting data check')
end

% Filtering
d_filt = designfilt('lowpassfir', ... % adjust the cutting freq values, based on the signal
'FilterOrder', 50,'PassBandFrequency', 4,'StopBandFrequency',18,... 
'DesignMethod','equiripple','SampleRate',1/0.001);
for j=1:n_cables            
    tensions_j_mean = mean(tensions(j,:));
    tensions_j_filt = tensions_j_mean + filtfilt(d_filt,tensions(j,:)-tensions_j_mean);
    tensions_r(j,:) = tensions_j_filt;
end

if show
    figure()
    hold on
    grid on
    plot(t_r,tensions(1,:))
    plot(t_r,tensions_r(1,:))
    legend('tension', 'tension filtered')
end

% Erasing invalid inclinometer data
eps_idx = 1;
for i = 2:length(epsilon_r)
    if epsilon_r(1,i)~=epsilon_r(1,i-1)
        eps_idx(end+1) = i; 
    end
end

if show
    figure()
    hold on
    grid on
    plot(t_r,epsilon_r(1,:))
    plot(t_r(eps_idx),epsilon_r(1,eps_idx))
    legend('epsilon', 'epsilon filtered')
end

end