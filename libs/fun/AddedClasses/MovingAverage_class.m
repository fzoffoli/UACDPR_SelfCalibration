classdef MovingAverage_class
    properties
        measures
        
        n
        
        out_value
        
    end
    
    methods
        function obj = MovingAverage_class(in,measures_init)
            obj.measures=measures_init;
            obj.n=in;
        end
        
        function obj=AddNewMeas(obj,new_meas)
            
            obj.measures(1:obj.n-1) = obj.measures(2:obj.n);
            obj.measures(obj.n) = new_meas;
            
            obj.out_value=mean(obj.measures);
        end
    
    end
end