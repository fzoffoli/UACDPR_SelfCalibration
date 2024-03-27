classdef InclinometerData < handle
    properties
        roll
        pitch
        yaw
    end
    
    methods
        function set(obj, inclinometer_data_packed)
            obj.roll = inclinometer_data_packed(1);
            obj.pitch = inclinometer_data_packed(2);
            obj.yaw = inclinometer_data_packed(3);
        end
        
        function append(obj, inclinometer_data_packed)
            obj.roll(end + 1) = inclinometer_data_packed(1);
            obj.pitch(end + 1) = inclinometer_data_packed(2);
            obj.yaw(end + 1) = inclinometer_data_packed(3);            
        end        
        
        function clear(obj)
            obj.roll = [];
            obj.pitch = [];
            obj.yaw = [];
        end
    end
end