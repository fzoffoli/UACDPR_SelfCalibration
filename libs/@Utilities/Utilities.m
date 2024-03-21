classdef Utilities
    %UTILITIES contains utilities supporting other class, e.g. solutor
    %options
    
    properties
        FsolveEqPoses
        FsolveStartPose

        FsolveNoGrad8
        FsolveGradCheck8
        FsolveGradCheck10
        FsolveGrad8
        FsolveNoGradDetail8
        FsolveNoGrad2
        FsolveGrad2
        
        Ode4
        Ode6
        Ode8
        
        fmincon_options
        extreme_fmincon_options
        brutal_fmincon_options
        brutal_fmincon_options_nopar
        no_par_fmincon_options
        fmincon_options_grad

        omega_baum
        zita_baum
    end
    


%%% CONSTRUCTOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
        function obj = Utilities()
            % Sara
            obj.FsolveEqPoses=optimoptions('fsolve','Display','none','FunctionTolerance',1e-10,'MaxFunctionEvaluation',1000000,'MaxIterations',1000000,'OptimalityTolerance',1e-10,'SpecifyObjectiveGradient',false,'StepTolerance',1e-10); 
            obj.FsolveStartPose=optimoptions('fsolve','Display','none','FunctionTolerance',1e-16,'MaxFunctionEvaluation',1000000,'MaxIterations',1000000,'OptimalityTolerance',1e-16,'SpecifyObjectiveGradient',false,'StepTolerance',1e-16); 
            % Michele
            obj.FsolveNoGrad8=optimoptions('fsolve','FunctionTolerance',1e-8,'StepTolerance',1e-8,'SpecifyObjectiveGradient',false,'Algorithm','levenberg-marquardt');
            obj.FsolveGradCheck8=optimoptions('fsolve','FunctionTolerance',1e-8,'StepTolerance',1e-8,'SpecifyObjectiveGradient',true,'CheckGradient',true);
            
            obj.FsolveGradCheck10=optimoptions('fsolve','FunctionTolerance',1e-10,'StepTolerance',1e-10,'SpecifyObjectiveGradient',true);
            obj.FsolveGrad8=optimoptions('fsolve','FunctionTolerance',1e-8,'StepTolerance',1e-8,'SpecifyObjectiveGradient',true,'Display','off','Algorithm','levenberg-marquardt');
            
            obj.FsolveNoGradDetail8=optimoptions('fsolve','FunctionTolerance',1e-8,'StepTolerance',1e-8,'SpecifyObjectiveGradient',false,'Display','iter-detailed');
            
            obj.FsolveNoGrad2=optimoptions('fsolve','FunctionTolerance',1e-3,'StepTolerance',1e-6,'SpecifyObjectiveGradient',false);
            obj.FsolveGrad2=optimoptions('fsolve','FunctionTolerance',1e-3,'StepTolerance',1e-6,'SpecifyObjectiveGradient',true);
            
            obj.Ode4= odeset('RelTol',1e-4,'AbsTol',1e-6);
            obj.Ode6= odeset('RelTol',1e-6,'AbsTol',1e-6);
            obj.Ode8= odeset('RelTol',1e-8,'AbsTol',1e-8,'InitialStep',1e-3);
            
            obj.omega_baum=15;
            obj.zita_baum=1;

            % Filippo
            obj.fmincon_options = optimoptions('fmincon','Algorithm','interior-point',...
            'FunctionTolerance',1e-8,'MaxFunctionEvaluation',1000000,...
            'MaxIterations',1000000,'OptimalityTolerance',1e-6,'StepTolerance',1e-16,'UseParallel',true,'ConstraintTolerance',1e-6);

            obj.extreme_fmincon_options = optimoptions('fmincon','Algorithm','interior-point',...
            'FunctionTolerance',1e-14,'MaxFunctionEvaluation',1000000,...
            'MaxIterations',1000000,'OptimalityTolerance',1e-14,'StepTolerance',1e-14,'UseParallel',true,'ConstraintTolerance',1e-11);
            
            obj.brutal_fmincon_options = optimoptions('fmincon','Algorithm','interior-point',...
            'FunctionTolerance',1e-2,'MaxFunctionEvaluation',100,... 
            'MaxIterations',100,'OptimalityTolerance',1e-2,'StepTolerance',1e-3,'UseParallel',true,'ConstraintTolerance',1e-3);

            obj.brutal_fmincon_options_nopar = optimoptions('fmincon','Algorithm','interior-point',...
            'FunctionTolerance',1e-2,'MaxFunctionEvaluation',1000,...
            'MaxIterations',100,'OptimalityTolerance',1e-2,'StepTolerance',1e-3,'UseParallel',false,'ConstraintTolerance',1e-3);

            obj.no_par_fmincon_options = optimoptions('fmincon','Algorithm','interior-point',...
            'FunctionTolerance',1e-8,'MaxFunctionEvaluation',1000000,...
            'MaxIterations',1000000,'OptimalityTolerance',1e-6,'StepTolerance',1e-8,'ConstraintTolerance',1e-14);

            obj.fmincon_options_grad = optimoptions('fmincon','Algorithm','interior-point',...
            'FunctionTolerance',1e-8,'MaxFunctionEvaluation',1000000,...
            'MaxIterations',1000000,'OptimalityTolerance',1e-2,'StepTolerance',1e-8,...
            'UseParallel',true,'ConstraintTolerance',1e-6,'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true);
        end
    end
%%% GETTERs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% PRIVATE METHODS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end

