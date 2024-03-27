classdef PlatformStatus < handle
    properties
        position_x
        position_y
        position_z
        Roll
        Pitch
        Yaw
        target_position_x
        target_position_y
        target_position_z
        omega_x
        omega_y
        omega_z
        emptybox1
        emptybox2
        emptybox3

    end
    
    methods
        function set(obj, platform_status_packed)
            obj.position_x = platform_status_packed(1);
            obj.position_y = platform_status_packed(2);
            obj.position_z = platform_status_packed(3);
            obj.Roll=platform_status_packed(4);
            obj.Pitch=platform_status_packed(5);
            obj.Yaw=platform_status_packed(6);
            obj.target_position_x = platform_status_packed(7);
            obj.target_position_y = platform_status_packed(8);
            obj.target_position_z = platform_status_packed(9);
            obj.omega_x=platform_status_packed(10);
            obj.omega_y=platform_status_packed(11);
            obj.omega_z=platform_status_packed(12);
            obj.emptybox1=platform_status_packed(13);
            obj.emptybox2=platform_status_packed(14);
            obj.emptybox3=platform_status_packed(15);

        end
        
        function append(obj, platform_status_packed)
            obj.position_x(end + 1) = platform_status_packed(1);
            obj.position_y(end + 1) = platform_status_packed(2);
            obj.position_z(end + 1) = platform_status_packed(3);
            obj.Roll(end + 1)=platform_status_packed(4);
            obj.Pitch(end + 1)=platform_status_packed(5);
            obj.Yaw(end + 1)=platform_status_packed(6);
            obj.target_position_x (end + 1)= platform_status_packed(7);
            obj.target_position_y (end + 1)= platform_status_packed(8);
            obj.target_position_z (end + 1)= platform_status_packed(9);
            obj.omega_x(end + 1)=platform_status_packed(10);
            obj.omega_y(end + 1)=platform_status_packed(11);
            obj.omega_z(end + 1)=platform_status_packed(12);
            obj.emptybox1(end + 1)=platform_status_packed(13);
            obj.emptybox2(end + 1)=platform_status_packed(14);
            obj.emptybox3(end + 1)=platform_status_packed(15);

            
        end
        
        function clear(obj)
            obj.position_x = [];
            obj.position_y = [];
            obj.position_z = [];
            obj.Roll = [];
            obj.Pitch = [];
            obj.Yaw = [];
            obj.target_position_x = [];
            obj.target_position_y = [];
            obj.target_position_z = [];
            obj.omega_x=[];
            obj.omega_y=[];
            obj.omega_z=[];
            obj.emptybox1(end+1)=[];
            obj.emptybox2(end + 1)=[];
            obj.emptybox3(end + 1)=[];

            
        end
    end
end
