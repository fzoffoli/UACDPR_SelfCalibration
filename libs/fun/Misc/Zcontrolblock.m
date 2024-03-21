classdef Zcontrolblock
    % Class Description
    
    properties (SetAccess= private)
        num
        den
        dt
        
        
        input_weights
        output_weights
        
        output
        input
        
        history_input
        history_output
        t_prev
    end
    

%%% PUBLIC METHODS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods

        %Brief Description
        function obj = addSample(obj,input,t)
            obj.input=input;
            if t-obj.t_prev<obj.dt
            else
                obj.t_prev=t;
                obj.history_input(1:end-1)=obj.history_input(2:end);
                obj.history_input(end)=input;
                obj.output=obj.input_weights*obj.history_input-obj.output_weights*obj.history_output;
                obj.history_output(1:end-1)=obj.history_output(2:end);
                obj.history_output(end)=obj.output;
            end
            
        end

    end 
    
%%% CONSTRUCTOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
        
        function obj=Zcontrolblock(num,den,dt)
            obj.dt=dt;
            obj.num=num/den(1);
            obj.den=den/den(1);
            
            obj.input_weights=flip(num);
            obj.output_weights=flip(obj.den(2:end));
            
            obj.history_input=zeros(length(obj.input_weights),1);
            obj.history_output=zeros(length(obj.output_weights),1);
            obj.t_prev=0;
            
            obj.output=0;
            
        end
        
    end
    
end

