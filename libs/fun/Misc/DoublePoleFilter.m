classdef DoublePoleFilter
    % Class for impementation of a double pole filter of the kind
%               K*s
%         ----------------
%          s^2+2*z*w*s+w^2

% Divided in three parts :
% integration---->first pole---->second pole
    
    properties (SetAccess= private)
        input
        output
        
        dt
        zita
        w0
        
        pole1
        pole2
        wd
        
        afterint
        afterfirstloop
        aftersecondloop
        
        FirstLoop
        SecondLoop
    end
    

%%% PUBLIC METHODS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods

        %Brief Description
        function obj = AddSample(obj,input)
        
        obj.input=input;
        obj.afterint=obj.afterint+input*obj.dt;
        
        obj.FirstLoop=obj.FirstLoop.AddSample(obj.afterint);
        obj.afterfirstloop=obj.FirstLoop.output;
        
        obj.SecondLoop=obj.SecondLoop.AddSample(obj.afterfirstloop);
        obj.aftersecondloop=obj.SecondLoop.output;
        
        obj.output=obj.aftersecondloop;
        
        end

    end 
    
%%% CONSTRUCTOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
        
        function obj=DoublePoleFilter(dt,zita,w0)
            obj.dt=dt;
            obj.w0=w0;
            obj.zita=zita;
            
            obj.wd=w0*sqrt(zita^2-1);
            obj.pole1=-zita*w0+obj.wd;
            obj.pole2=-zita*w0-obj.wd;
            
            obj.FirstLoop=ApplyIntegrationFeedback(dt,obj.pole1);
            obj.SecondLoop=ApplyIntegrationFeedback(dt,obj.pole2);
        end
        
    end
end

