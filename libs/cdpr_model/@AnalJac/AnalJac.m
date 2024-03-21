classdef AnalJac
    % Geometric Jacobians container
    
    properties (SetAccess= ?UACDPR)
        Cables
        Cables_dt
        Swivel double
        Tangency double
        
        Cables_P double        %Permuted Geometric Jacobian
        Cables_ort  double     %ortogonal Geometric cable Jacobian
        Cables_par  double     %parallel Geometric cable Jacobian
        
        Cables_Port  double     %ortogonal Geometric cable Jacobian
        Cables_Ppar  double     %parallel Geometric cable Jacobian
        
        Cables_ort_dt double   %ortogonal Geometric cable Jacobian first time derivative
        Cables_par_dt double   %parallel Geometric cable Jacobian first time derivative
        
        Cables_Port_dt
        Cables_Ppar_dt
        
        Cables_P_dt double      %Permuted Geometric Jacobian time derivative
    end
    
%%% PUBLIC METHODS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%%% CONSTRUCTOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
        
        function obj=AnalJac(DepVect)
            obj.Cables_dt=zeros(nnz(DepVect),6);
            obj.Cables=zeros(nnz(DepVect),6);
        end
        
    end
    
%%% GETTERs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    
%%% PRIVATE METHODS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end

