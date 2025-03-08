% This function filter the data of the parsed .log file from the app
% cdpr_alars
% INPUT parsed log data from file .log
% OUTPUT time vector, RPY vector with erased repetitions, cable lengths
% vector, swivel angles vector, tension vector filtered, index of epsilon
% wrt others vectors, yaw calibration angle.

function [t_r,epsilon_r,cable_length_r,swivel_r,tensions_r,target_tensions_r] = CdprLogProcessing(log_data,n_cables,homing_val)

% Inclinometer data extraction
t = log_data.inclinometer_data.timestamp;
epsilon = [log_data.inclinometer_data.values.roll;        
log_data.inclinometer_data.values.pitch;
log_data.inclinometer_data.values.yaw];

% Actuators data extraction
invalid_data = mod(length(log_data.actuator_status.timestamp),n_cables);
n_acquisitions = (length(log_data.actuator_status.timestamp)-invalid_data)/n_cables;
cable_length = reshape(log_data.actuator_status.values.cable_length,[n_cables n_acquisitions]);
swivel = reshape(-log_data.actuator_status.values.pulley_angle,[n_cables n_acquisitions]); %NB the sign of the swivels was opposite in the config file
tensions = reshape(log_data.actuator_status.values.cable_tension,[n_cables n_acquisitions]);
target_tensions = reshape(log_data.actuator_status.values.target,[n_cables n_acquisitions]);

% NB the transmission ratio was wrong in the config file, here the cable
% lenght values are corrected:
cable_length = FixActuatorsTransmissionRatio(cable_length);

% Homing relative sensor output computation
for i = 2:length(t)
    if (t(i)-t(i-1))>2 %[s] assuming a min logging stop of 2 seconds
        n_homing = i-1;
        break
    end
end
length_home = mean(cable_length(:,1:n_homing),2);
swivel_home = mean(swivel(:,1:n_homing),2);
yaw_home = mean(epsilon(3,1:n_homing));

% Cutting data with the experiment logging time
epsilon_r = epsilon(:,n_homing+1:end)-[0;0;yaw_home]; % assuming yaw zero in home
swivel_r = swivel(:,n_homing+1:end)-swivel_home+homing_val.swivels;
cable_length_r = cable_length(:,n_homing+1:end)-length_home+homing_val.lengths;
tensions_r = tensions(:,n_homing+1:end);
target_tensions_r = target_tensions(:,n_homing+1:end);
t_r = t(n_homing+1:end);

end