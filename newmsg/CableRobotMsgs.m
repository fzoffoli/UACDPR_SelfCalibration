classdef CableRobotMsgs < uint32
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    enumeration
        NONE(0)
        MOTOR_STATUS(1)
        WINCH_STATUS(2)
        ACTUATOR_STATUS(3)
        PLATFORM_STATUS(4)
        INCLINOMETER_DATA(5)
        LOGGING_EXTRAS(6)
    end
end
