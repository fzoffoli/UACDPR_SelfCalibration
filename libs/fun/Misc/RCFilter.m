classdef RCFilter
    % Class Description
    
    properties (SetAccess= private)
        input
        output
        
        prev_value
        
        freq
        w
        dt
        
        alfa
        gain
        
    end
    

%%% PUBLIC METHODS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods

        %Brief Description
        function obj = AddSample(obj,input)
            obj.input=input;
            obj.prev_value=obj.output;
            obj.output=obj.alfa*obj.input*obj.gain+(1-obj.alfa)*obj.prev_value;           
            
        end
        
        function obj = ChangeGain(obj,gain)
            obj.gain=gain;
        end
        
    end 
    
%%% CONSTRUCTOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
        
        function obj=RCFilter(freq,dt,x0)
            obj.input=0;
            obj.output=x0;
            obj.prev_value=0;
            obj.freq=freq;
            obj.w=2*pi*freq;
            obj.dt=dt;
            obj.alfa=obj.w*dt/(1+obj.w*dt);
            obj.gain=1;
            
        end
        
    end
   

end

