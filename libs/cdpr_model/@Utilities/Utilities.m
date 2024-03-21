classdef Utilities
    %UTILITIES contains utilities supporting other class, e.g. solutor
    %options
    
    properties
        FsolveNoGrad8
        FsolveGradCheck8
        FsolveGrad8
    end
    


%%% CONSTRUCTOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
        function obj = Utilities()
            obj.FsolveNoGrad8=optimoptions('fsolve','FunctionTolerance',1e-8,'StepTolerance',1e-8,'SpecifyObjectiveGradient',false);
            obj.FsolveGradCheck8=optimoptions('fsolve','FunctionTolerance',1e-8,'StepTolerance',1e-8,'SpecifyObjectiveGradient',true,'CheckGradient',true);
            obj.FsolveGrad8=optimoptions('fsolve','FunctionTolerance',1e-8,'StepTolerance',1e-8,'SpecifyObjectiveGradient',true);

        end
    end
%%% GETTERs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% PRIVATE METHODS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end

