function [messages, num_msgs] = parseCableRobotLogFile(filename, show)
%% Safety checks
if ~endsWith(filename, '.log')
    error('Error: invalid input file type. Please load a ".log" file.')
end

fid = fopen(filename, 'r');
if fid < 0
    error('Error: could not open file %s', filename)
end

%% Init
messages = initMessages();
counter_empty = 0;
counter_unknown = 0;

%% Read file
while 1
    tline = fgetl(fid);
    if ~ischar(tline)
        break;
    end
    split_line = str2num(tline(1:end)); %#ok<ST2NM>
    switch split_line(1)
        case CableRobotMsgs.NONE                    % 0
            counter_empty = counter_empty + 1;
        case CableRobotMsgs.MOTOR_STATUS            % 1
            messages.motor_status.timestamp(end + 1) = split_line(2);
            messages.motor_status.values.append(split_line(3:end));
        case CableRobotMsgs.WINCH_STATUS            % 2
            messages.winch_status.timestamp(end + 1) = split_line(2);
            messages.winch_status.values.append(split_line(3:end));
        case CableRobotMsgs.ACTUATOR_STATUS         % 3
            messages.actuator_status.timestamp(end + 1) = split_line(2);
            messages.actuator_status.values.append(split_line(3:end));
        case CableRobotMsgs.PLATFORM_STATUS         % 4
            messages.platform_status.timestamp(end + 1) = split_line(2);
            messages.platform_status.values.append(split_line(3:end));
        case CableRobotMsgs.INCLINOMETER_DATA       % 5
            messages.inclinometer_data.timestamp(end + 1) = split_line(2);
            messages.inclinometer_data.values.append(split_line(3:end));
        case CableRobotMsgs.LOGGING_EXTRAS
            messages.logging_extras.timestamp(end + 1) = split_line(2);
            messages.logging_extras.values.append(split_line(3:end));
        otherwise
            counter_unknown = counter_unknown + 1;
    end    
end

fclose(fid);

%% Count messages
num_msgs.total = 0;
msg_types = fields(messages);
for i = 1:length(msg_types)
    message_len = length(messages.(msg_types{i}).timestamp);
    if message_len == 0
        messages = rmfield(messages, msg_types{i});
        continue
    end
    num_msgs.(msg_types{i}) = message_len;
    num_msgs.total = num_msgs.total + num_msgs.(msg_types{i});
end
num_msgs.empty = counter_empty;
num_msgs.unknown = counter_unknown;
num_msgs.total = num_msgs.total + counter_empty + counter_unknown;

%% Plot
if nargin == 1 || ~show
    return
end
msg_types = fields(messages);
for i = 1:length(msg_types)
    msg_fields = fields(messages.(msg_types{i}).values);
    time = messages.(msg_types{i}).timestamp;
    figure;
    hold on
    for j = 1:length(msg_fields)
        plot(time, messages.(msg_types{i}).values.(msg_fields{j}))
        msg_fields{j} = replace(msg_fields{j}, '_', '\_');
    end
    legend(msg_fields)
    title(replace(msg_types{i}, '_', '\_'))
    xlabel('time [sec]')
    xlim([time(1) time(end)])
    hold off
end

end

function messages = initMessages()
    messages.motor_status = struct('timestamp', [], 'values', MotorStatus);
    messages.winch_status = struct('timestamp', [], 'values', WinchStatus);
    messages.actuator_status = struct('timestamp', [], ...
                                      'values', ActuatorStatus);
    messages.platform_status = struct('timestamp', [], 'values', PlatformStatus);
    messages.inclinometer_data = struct('timestamp', [], ...
                                      'values',InclinometerData);
    messages.logging_extras = struct('timestamp', [], ...
                                      'values',ExtraData);
end
