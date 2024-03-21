classdef Platform
    %UNTITLED7 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        AttachPoint cell        %Cell array containing Attach points (P-A)'
        PtoG (3,1) double       %Reference point to center of mass vector
        
        Mass (1,1) double
        BarycentricInertia (3,3) double %Local Inertia Matrix
    end


%%% PUBLIC METHODS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%%% CONSTRUCTOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods %(Access= ?EndEffector)
        
        function obj = Platform(In)
            %Constructor

                obj.Mass=In.m;
                obj.BarycentricInertia=In.InertialMatrix;
                obj.PtoG=In.PtoG;
            
                for i=1:length(In.points)
                obj.AttachPoint{i}=In.points{i}';
                end
        end
    end
    
%%% GETTERs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% PRIVATE METHODS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   

end

