classdef LowPassFilter_class
    
    properties
    out_value    
        
    past_value
    now_value
    last_meas
    
    freq
    dt
    
    end
    methods
        function obj = LowPassFilter_class(freq,dt)
            obj.freq=freq;
            obj.dt=dt;
        end
        
        function obj = AddNewMeas(obj,new_meas)
           obj.last_meas=new_meas;
           new_value=MyLowPassFilt(new_meas,obj.now_value,1,obj.dt,obj.freq);
           
           obj.past_value=obj.now_value;
           obj.now_value=new_value;
           obj.out_value=new_value;
            
        end
        
    end
end