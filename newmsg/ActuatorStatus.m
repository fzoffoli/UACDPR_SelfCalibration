classdef ActuatorStatus < handle
    properties
        id
        op_mode
        motor_position
        motor_speed
        motor_torque
        cable_length
        cable_speed
        cable_tension
        target
        aux_position
        state
        pulley_angle
    end
    
    methods
        function set(obj, actuator_status_packed)
            obj.id = actuator_status_packed(1);
            obj.op_mode = actuator_status_packed(2);
            obj.motor_position = actuator_status_packed(3);
            obj.motor_speed = actuator_status_packed(4);
            obj.motor_torque = actuator_status_packed(5);
            obj.cable_length = actuator_status_packed(6);
            obj.cable_speed = actuator_status_packed(7);
            obj.cable_tension = actuator_status_packed(8);
            obj.target = actuator_status_packed(9);
            obj.aux_position = actuator_status_packed(10);
            obj.state = actuator_status_packed(11);
            obj.pulley_angle = actuator_status_packed(12);
        end
        
        function append(obj, actuator_status_packed)
            obj.id(end + 1) = actuator_status_packed(1);
            obj.op_mode(end + 1) = actuator_status_packed(2);
            obj.motor_position(end + 1) = actuator_status_packed(3);
            obj.motor_speed(end + 1) = actuator_status_packed(4);
            obj.motor_torque(end + 1) = actuator_status_packed(5);
            obj.cable_length(end + 1) = actuator_status_packed(6);
            obj.cable_speed(end + 1) = actuator_status_packed(7);
            obj.cable_tension(end + 1) = actuator_status_packed(8);
            obj.target(end + 1) = actuator_status_packed(9);
            obj.aux_position(end + 1) = actuator_status_packed(10);
            obj.state(end + 1) = actuator_status_packed(11);
            obj.pulley_angle(end + 1) = actuator_status_packed(12);
            
        end        
        
        function clear(obj)
            obj.id = [];
            obj.op_mode = [];
            obj.motor_position = [];
            obj.motor_speed = [];
            obj.motor_torque = [];
            obj.cable_length = [];
            obj.cable_speed=[];
            obj.cable_tension = [];
            obj.target=[];
            obj.aux_position = [];
            obj.pulley_angle = [];
            obj.state = [];
        end
    end
end
