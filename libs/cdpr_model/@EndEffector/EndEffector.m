classdef EndEffector
    %Class End Effector:
    %Describes End Effector parameters and variables
    
    properties (SetAccess = private)

        RotMatrix (3,3) double   %Rotational Matrix 
        RotMatrix_dpose (3,3,6) double  %Pose derivative of rotational matrix
        H (3,3) double           %H Matrix
        D (6,6) double           %D Matrix
        
        H_dt (3,3) double        %H Matrix time derivative (TBD)
        D_dt (6,6) double        %D Matrix time derivative (TBD)

        H_dpose (3,3,6) double   %H Matrix pose derivative
        D_dpose (6,6,6) double   %H Matrix pose derivative
        
        InertiaMatrix (3,3) double
        M (6,6)double            %Equivalent Mass Matrix (TBD)
        C (6,6)double            %Equivalent Damping Matrix (TBD)   
        
        Pose (6,1) double        %Pose
        Pose_dt (6,1) double     %Pose first ord time derivative
        Pose_ddt (6,1) double    %Pose second ord time derivative
        
        Platform_Param Platform        %Platform Parameters
        GlobalAttachPoints cell  %Global referenced attach points (O-A)
        
        PtoG_global (3,1) double %Rotated PtoG vector
 
    end
    properties
       OrientType  string
    end
    
%%% PSEUDO Properties %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    

    methods 
        function a = Coord_(obj)
           a=obj.Pose(1:3); 
        end
        
        function a = Orient_(obj)
           a=obj.Pose(4:6); 
        end
        
        function a = LocalAttachPoints_(obj)
            a=obj.GlobalAttachPoints;
            for i=1:length(obj.GlobalAttachPoints)
                a{i}=obj.GlobalAttachPoints{i}-obj.Pose(1:3); 
            end
        end
    end

    
%%% PUBLIC METHODS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    methods
        %Update End Effector Geometric variables
       obj = UpdateEndEffector0KIN(obj,Pose) 
       %INPUT: Pose
       %OUTPUT: Set new pose, Update Rotational Matrix, Attach-points global coordinates and HD Matrixes 
       
       %Update End Effector First order kinematic variables
       obj = UpdateEndEffector1KIN(obj,Pose_dt,Twist) 
       %INPUT: Pose
       %OUTPUT: Set new pose, Update Rotational Matrix, Attach-points global coordinates and HD Matrixes 
       
       
       %Update End Effector Second order kinematic variables
       obj = UpdateEndEffector2KIN(obj,Pose_ddt) 
       %INPUT: Pose_ddt
       %OUTPUT: Set new pose_ddt 
        
       %Update Rotation matrix derivative wrt pose
       obj = ComputeRotMatrix_dpose(obj)
       
       %Update H matrix derivative wrt pose
       obj = ComputeHD_dpose(obj)
    end

    
%%% CONSTRUCTOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    methods %(Access = ?UACDPR)
        function EndEffector = EndEffector(In)
            EndEffector.Platform_Param = Platform(In.Platform);
            a=[0;0;0];
            
            for i=1:length(EndEffector.Platform_Param.AttachPoint)
                EndEffector.GlobalAttachPoints{i}=a;
            end
        end
    end
    
%%% GETTERs   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

           
%%% PRIVATE METHODS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    methods (Access= private)
%         %Computation of the rotational matrix Yaw/Pitch/Roll
%         obj = RotMatrixCompYPR(obj)
%         %INPUT: Pose
%         %OUTPUT: Rotational Matrix
%               
%         %Computation of the rotational matrix as per PhD Edo Chapter 2.2
%         obj = RotMatrixCompTiltTorsion(obj)
%         %INPUT: Pose
%         %OUTPUT: Rotational Matrix
%         
        %Computation of global-referenced platform attach point
        EndEffector = GlobalComp(EndEffector)
        %INPUT: Attach point Id (none will compute all the attach points)
        %OUTPUT: Attach point coordinates w.r.t. reference fixed frame
%                
%         %Computation of Matrix H and D Yaw/Pitch/Roll
%         obj = ComputeHDYPR(obj)
%         %INPUT: Orientation
%         %OUTPUT: H and D Matrices 
%         
%         %Computation of Matrix H and D as per PhD Edo Chapter 2.2
%         obj = ComputeHDTiltTorsion(obj)
%         %INPUT: Orientation
%         %OUTPUT: H and D Matrices 
        
        %Computation of Matrix M 
        EndEffector = ComputeM(EndEffector)
        %INPUT: Platform, Rotation Matrix
        %OUTPUT: Inertia tensor about P, M Matrix as per PhD Edo Chapter 2.3
        
        %Computation of Matrix C 
        EndEffector = ComputeC(EndEffector,Twist)
        %INPUT: PtoG_global, Platform Mass ,Inertia tensor about P
        %OUTPUT: Inertia tensor about P, C Matrix as per PhD Edo Chapter 2.3
     
    end
end

