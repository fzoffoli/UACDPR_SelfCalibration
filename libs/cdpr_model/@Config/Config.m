classdef Config
    %CONFIG This class only purpose is to make it possible to instantiate
    %an object for robot-parameters definition
    
    properties

    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% PUBLIC METHODS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods (Static)
%        EndEffector Creator
         s=CreateEndEffector(s)
%        INPUT: Struct
%        OUTPUT: Instantiation of EndEffector object
        

%        Trasmission Set Creator
         s=CreateTrasmissionSet(s,n)
%        INPUT: number of cables
%        OUTPUT: Instantiation of cables objects

         s=CreateKinVar(s)
        
         s=CreateDepVect(s)

%        Platform Creator
         newPlatform=CreatePlatform()
%        INPUT: (none) or platform object
%        OUTPUT:(dialog box for platform input) or substitution of platform
        
       
%        Pulley Set Creator
         newPulleySet=CreatePulleySet(n)
%        INPUT struct, number of pulleys
%        OUTPUT Set of pulley assignment to trasmission field
        
%        Create an external fixed wrench
         s = CreateWrench(s)
%        INPUT: struct
%        OUTPUT: Dialog box for wrench creation
        
    end

%%% CONSTRUCTOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    methods
        function obj = Config()
        end
    end
    
%%% GETTERs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% PRIVATE METHODS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   

end

