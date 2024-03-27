classdef SensorData < handle
    properties
        posx
        posy
        posz
        roll
        pitch
        yaw
        cov
        latest
    end
    
    methods
        function set(obj, sensor_data_packed)
            obj.posx=sensor_data_packed(1);
            obj.posy=sensor_data_packed(2);
            obj.posz=sensor_data_packed(3);
            obj.roll = sensor_data_packed(4);
            obj.pitch = sensor_data_packed(5);
            obj.yaw = sensor_data_packed(6);
            obj.cov = sensor_data_packed(7);
            obj.latest = sensor_data_packed(8);
        end
        
        function append(obj, sensor_data_packed)
            obj.posx(end + 1)=sensor_data_packed(1);
            obj.posy(end + 1)=sensor_data_packed(2);
            obj.posz(end + 1)=sensor_data_packed(3);
            obj.roll (end + 1)= sensor_data_packed(4);
            obj.pitch(end + 1) = sensor_data_packed(5);
            obj.yaw(end + 1) = sensor_data_packed(6);
            obj.cov(end + 1) =sensor_data_packed(7);
            obj.latest(end + 1) = sensor_data_packed(8);
        end        
        
        function clear(obj)
            obj.posx = [];
            obj.posy = [];
            obj.posz = [];
            obj.roll = [];
            obj.pitch = [];
            obj.yaw = [];
            obj.cov = [];
            obj.latest = [];
        end
    end
end