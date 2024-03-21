classdef ApplyIntegrationFeedback
%  Class for easy implementation of feedback loop with integration
% frequency domain representation
%        s              1
%   ----------  = --------------
%       s+G           1+G/s


    properties (SetAccess= private)
        input
        output
        
        dt
        G
        
        integration
        feedback
    end
    
    properties (Dependent)
        DependentProperty1
    end

%%% PUBLIC METHODS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods

        %Brief Description
        function obj = AddSample(obj,input)
            obj.input=input;
            obj.output=obj.input-obj.feedback;
                        
            obj.integral=obj.integral+obj.input*obj.dt;
            
            obj.feedback=obj.integral*obj.G;                                   
        end

    end 
    
%%% CONSTRUCTOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
        
        function obj=ApplyIntegrationFeedback(dt,G)
            obj.input=0;
            obj.output=0;
            obj.dt=dt;
            obj.G=G;
            obj.feedback=0;
            obj.integration=0;
        end
        
    end
    
%%% GETTERs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

