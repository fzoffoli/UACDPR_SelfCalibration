classdef ExtraData < handle
    properties
        double_0
        double_1
        double_2
        double_3
        double_4
        double_5
        double_6
        double_7
        double_8
        double_9
    end
    
    methods
        function set(obj, extra_data_packed)
            obj.double_0 = extra_data_packed(1);
            obj.double_1 = extra_data_packed(2);
            obj.double_2 = extra_data_packed(3);
            obj.double_3 = extra_data_packed(4);
            obj.double_4 = extra_data_packed(5);
            obj.double_5 = extra_data_packed(6);
            obj.double_6 = extra_data_packed(7);
            obj.double_7 = extra_data_packed(8);
            obj.double_8 = extra_data_packed(9);
            obj.double_9 = extra_data_packed(10);
        end
        
        function append(obj, extra_data_packed)
            obj.double_0(end + 1) = extra_data_packed(1);
            obj.double_1(end + 1) = extra_data_packed(2);
            obj.double_2(end + 1) = extra_data_packed(3);
            obj.double_3(end + 1) = extra_data_packed(4);
            obj.double_4(end + 1) = extra_data_packed(5);
            obj.double_5(end + 1) = extra_data_packed(6);
            obj.double_6(end + 1) = extra_data_packed(7);
            obj.double_7(end + 1) = extra_data_packed(8);
            obj.double_8(end + 1) = extra_data_packed(9);
            obj.double_9(end + 1) = extra_data_packed(10);       
        end        
        
        function clear(obj)
            obj.double_0 = [];
            obj.double_1 = [];
            obj.double_2 = [];
            obj.double_3 = [];
            obj.double_4 = [];
            obj.double_5 = [];
            obj.double_6 = [];
            obj.double_7 = [];
            obj.double_8 = [];
            obj.double_9 = [];
        end
    end
end