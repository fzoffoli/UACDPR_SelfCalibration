classdef Cable
    %UNTITLED14 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess= private)

        CableVector         (3,1) double           % rho

        Length              (1,1) double           % l = norm(rho)+arco
        Length_dpose        (1,6) double
        
        Id                  (1,1) int8      

        ModelVectors        struct
        ModelVectors_dpose  struct
        
    end
%     properties (SetAccess=?Trasmission)
%             Tension (1,1) double          %Cable Tension (TBD)
%     end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
     
%%% PUBLIC METHODS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
        %Update Cable
        obj=UpdateCable(obj,AttachPoint,CableExitPosition,PulleyRadius,PulleyTangencyAngle)
        %INPUT Cable Exit Position, Global Platform Attach point, Tangency
        %angle and Pulley Radius
        %OUTPUT Update Cable Vector info and Cable Length 

        obj=UpdateModelVectors(obj,UACDPR)
        obj=UpdateModelVectors_dpose(obj,UACDPR)

        obj=ComputeCable_dpose(obj,UACDPR)
    end

%%% CONSTRUCTOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    methods %(Access= ?Trasmission)
        function obj = Cable(Id)
            %Instantiate object with Id
            if nargin==1
                obj.Id=Id;

                ModelVectors.ap = zeros(3,1);
                ModelVectors.varrho = zeros(3,1);
                ModelVectors.varrho_u = zeros(1,1);
                ModelVectors.rho = zeros(3,1);

                ModelVectors_dpose.ap = zeros(3,6);
                ModelVectors_dpose.varrho = zeros(3,6);
                ModelVectors_dpose.varrho_u = zeros(1,6);
                ModelVectors_dpose.rho = zeros(3,1);

                obj.ModelVectors= ModelVectors;
                obj.ModelVectors_dpose= ModelVectors_dpose;
            else
                ModelVectors.ap = zeros(3,1);
                ModelVectors.varrho = zeros(3,1);
                ModelVectors.varrho_u = zeros(1,1);
                ModelVectors.rho = zeros(3,1);

                ModelVectors_dpose.ap = zeros(3,6);
                ModelVectors_dpose.varrho = zeros(3,6);
                ModelVectors_dpose.varrho_u = zeros(1,6);
                ModelVectors_dpose.rho = zeros(3,1);
                ModelVectors_dpose.norm_rho = zeros(1,6);
                ModelVectors_dpose.inv_norm_rho = zeros(1,6);

                obj.ModelVectors= ModelVectors;
                obj.ModelVectors_dpose= ModelVectors_dpose;
            end
        
        end
    end  
    
%%% GETTERs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% PRIVATE METHODS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   

end

