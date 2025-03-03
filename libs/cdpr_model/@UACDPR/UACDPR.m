classdef UACDPR
    % Unactuated Robot Class
    %%%% TO DRAW ROBOT CALL FOR FUNCTION FilmDrawRobot(Pose_Plist,obj)
    properties (SetAccess = private)
        
        
        EndEffector EndEffector         %End Effector related variables and parameters
        Trasmission Trasmission         %Trasmission (winch/cable/pulley) related variables and parameters   

        %%% Parameters
        CablesNumber int32              %Number of cables
        FreeNumber int32                %6 - Number of cables
        
        DependencyVect (1,6) double     %Vector controlling dependent variables and free variables
        PermutMatrix (6,6) double       %Permutation Matrix
        
        GlobalForces    struct          %Structs containing config-initialized forces and their application points, 
        LocalForces     struct          %each column of forces matrix is a force associated with the point whose coordinates are given 
%                                           by the first column of the points matrix:
%                                            -Global struct contains forces fixed w.r.t. fixed frames
%                                            -Local struct contains forces fixed w.r.t. end effector Frame
 
        %%%%%%%
        GlobalMoments (3,1) double      %Constant moments from config and from LocalForce                                            

        Wrench     (6,1) double         %Full external Wrench
        
        GeomJac GeomJac
        AnalJac AnalJac

        GeomJac_dpose GeomJac_dpose
        AnalJac_dpose AnalJac_dpose
  
        %%% Dynamic equation related variables
        Twist (6,1) double          %Twist 
        Twist_dt (6,1) double       %Twist time derivative
        
        M_Ppar double               %Parallel permuted equivalent mass matrix  
        M_Port double               %Ortogonal permuted equivalent mass matrix
        
        C_Ppar double               %Parallel permuted equivalent damping matrix
        C_Port double               %Ortogonl permuted equivalent damping matrix
        
        %%% Stiffness related variables        
        ActiveStiffness (6,6) double    %Active/Geometric Stiffness
        MatrixE (6,6) double            %PhD Edo Chapter 2.4 
        MatrixF (6,6) double            %PhD Edo Chapter 2.4  
        CMS double                      %Controlled Motion Stiffness
        FMS double                      %Free Motion Stiffness   
        
        test_att 
    end
    

%%% PSEUDO Properties %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
methods        
        function [a,obj]=test_meth(obj)
                a=obj.Trasmission.CableLengths*2;
                obj.test_att=a;
        end

        function a=CableLengths_(obj)  
              a=obj.Trasmission.CableLengths;     
        end
        
        function a=Pose_d_(obj)
            a=obj.EndEffector.Pose(1:nnz(obj.DependencyVect));
        end
        
        function a=Pose_f_(obj)
            a=obj.EndEffector.Pose((nnz(obj.DependencyVect)+1):end);
        end
        
        function a=Pose_P_(obj)
            a=obj.PermutMatrix*obj.EndEffector.Pose;
        end 
        
        function a=Pose_(obj)
            a=obj.EndEffector.Pose;
        end 
        
        function a=GeomJac_Port_(obj)
            a=obj.PermutMatrix*obj.GeomJacl_ort;
        end
        
        function a=GeomJac_Ppar_(obj)
            a=obj.PermutMatrix*obj.GeomJacl_par;
        end
        
        function a=AnalJacl_ort_(obj)
            a=obj.PermutMatrix'*obj.AnalJac_Port;
        end
        
        function a=AnalJacl_par_(obj)
            a=obj.PermutMatrix'*obj.AnalJac_Ppar;
        end

        function a=CableTensions_(obj)
            a=obj.Trasmission.CableTensions;
        end
        
        function a=GeomJacl_(obj)
            a=obj.GeomJac.Cables;
        end
        
        function a=AnalJacl_(obj)
            a=obj.AnalJac.Cables;
        end
        
        function a=RotMatrix_(obj)
            a=obj.EndEffector.RotMatrix;
        end
        
        function a=GlobalAttachPoints_(obj)
            a=obj.EndEffector.GlobalAttachPoints;
        end
        
        function a=D_(obj)
            a=obj.EndEffector.D;
        end
        
        function a=H_(obj)
            a=obj.EndEffector.H;
        end
        
         function a=M_(obj)
            a=obj.EndEffector.M;
        end
        
        function a=C_(obj)
            a=obj.EndEffector.C;
        end
        
        function a=D_dt_(obj)
            a=obj.EndEffector.D_dt;
        end
        
        function a=H_dt_(obj)
            a=obj.EndEffector.H_dt;
        end
        
        function a=Pose_dt_(obj)
            a=obj.EndEffector.Pose_dt;
        end
        
        function a=GeomJacl_dt_(obj)
            a=obj.PermutMatrix*obj.GeomJac.Cables_dt;
        end
        
        function a=AnalJacl_dt_(obj)
            a=obj.PermutMatrix*obj.AnalJac.Cables_dt;
        end
    end


%%% PUBLIC METHODS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    methods
       
%        Set a Pose and Update Trasmission and End Effector geometric
%        variables and first order jacobians
        obj = SetPoseAndUpdate0KIN(obj,Pose)
        
%         Set Pose and its derivative and Update Trasmission and End Effector kinematic variables
        obj = SetPoseAndUpdate1KIN(obj,Pose,Pose_dt)
        
%         Set Pose and its derivatives and Update Trasmission and End Effector kinematic variables 
        obj = SetPoseAndUpdate2KIN(obj,Pose,Pose_dt,Pose_ddt)
        
%         Sum wrenches contributions
        obj = ComputeWrench(obj,Disturb)

%         Compute only gravity wrench
        obj = ComputeGravityWrench(obj,Disturb)
        
%         Compute Active/Geometric Stiffness
        obj = ComputeActiveStiffness(obj,ExternalForces)
%        PREREQs: obj.SetPosePAndUpdateGeom, Cable tensions
        
%         Compute Cable tension in static conditions
        obj = SetStaticCableTensions(obj)
%         PREREQs: obj.ComputeWrench, obj.ComputeJaclOrtPar
        
%         Compute ortogonal and parallel analitic AND geometric jacobians matrixes
        obj = ComputeJaclOrtPar(obj)
%        PREREQs: obj.SetPosePAndUpdateGeom//obj.SetPoseAndUpdateKin

        obj = ComputeJaclOrtPar_modified(obj)% modified to work for a 5cable cdpr as a 4 cable cdpr

%         Compute Controlled motion stiffness and free motion stiffness
        obj = ComputeCMSFMS(obj)
%         PREREQs: obj.ComputeActiveStiffness
        
%         Compute Controlled motion stiffness and free motion stiffness:
%         this version make use of the projection on the geometric jacobian
%         space to avoid parametrization singularity problems. As such it
%         works ONLY when no external forces are employed (Matrix F must be null)
        obj = ComputeCMSFMS_Hack(obj)
%         PREREQs: obj.ComputeActiveStiffness


%         Compute Jacobian derivatives
        obj = ComputeJaclOrtPar_dt(obj)
%         PREREQs: obj.ComputeJaclOrtPar, obj.SetPoseAndUpdateKin
        
%        Compute equivalent-mass-and-damping-matrix ortogonal and parallel matrixes
        obj = ComputeMC_Ppar(obj)
%         PREREQs: obj.ComputeJaclOrtPar, obj.SetPoseAndUpdateKin

%        Compute equivalent-mass-and-damping-matrix ortogonal and parallel matrixes
        obj = ComputeMC_Port(obj)
%         PREREQs: obj.ComputeJaclOrtPar, obj.SetPoseAndUpdateKin
        
%        Compute cable tension from pose and its time derivatives. Wrench must be computed separately        
        obj = ComputeTauInverseDynamic(obj,Pose,Pose_dt,Pose_ddt)
%         PREREQs: obj.ComputeWrench

%         Set Check on Cable tension to true
        obj = TauCheck(obj,LowerBound)
        
%         Compute Cable Lenghts derivatives given twist and twist
%         time derivative
        obj = ComputeLengthDerivatives(obj)
%         PREREQs: obj.SetPoseAndUpdate2KIN(Pose,Pose_dt,Pose_ddt)

%         Compute Natural Frequencies
        freq = ComputeNaturalFreq(obj,CableLengths,Guess_pose)
%         PREREQs: Cable length computation
        
%         Compute the Forward Geometrico-Static Problem to set the instance
%         in an equilibrium pose
        obj = ForwardGeoStaticProb(obj,CableLength,Disturb,Guess)
        
        %Set orient type
        obj = SetOrientType(obj,type)
        
         %Compute Cable Jacobians
        obj = UpdateJacl(obj,AttachPointsLocal,CableTs,D)
        %INPUT: End-Effector attach point, Cable-Frame First versor for each cable, D
        %OUTPUT: Analitic and Geometric cable Jacobians
        
        
        %Compute geometric and analitic jacobians time derivatives
        obj = UpdateJacl_dt(obj,Twist,LocalAttachPoints,FirstOrderKin,Pulley,Cable,D,D_dt)
        
        %Compute Angle Jacobians (geometric and analitic Swivel Angle jacobian, geometric and analitic Tangency Angle jacobian)
        obj = ComputeAngleJac(obj,LocalAttachPoints,Cable,Pulley,D)
        
        %Compute Angle Jacobians derivatives w.r.t. twist (geometric Swivel Angle jacobian, geometric Tangency Angle jacobian)
        obj = ComputeAngleJac_dt(obj,LocalAttachPoints,Cable,Pulley,D)

        %Compute derivatives of Geometric Jacobians w.r.t pose
        obj = ComputeJac_dpose(obj,LocalAttachPoints,Pulley)
        
        obj= CorrectGeometry(obj,Pose_frame)
            end   
    
%%% CONSTRUCTOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    methods
        function obj = UACDPR(s)
            if nargin ==1
                obj.EndEffector=EndEffector(s.EndEffector);
                obj.Trasmission=Trasmission(s.Trasmission);
                obj.CablesNumber=length(s.Trasmission.Cable);
                obj.GeomJac=GeomJac(s.DependencyVect);
                obj.AnalJac=AnalJac(s.DependencyVect);
                obj.GeomJac_dpose=GeomJac_dpose(s.DependencyVect);
                obj.AnalJac_dpose=AnalJac_dpose(s.DependencyVect);
                obj.DependencyVect=s.DependencyVect;
                obj.GlobalForces=s.GlobalForces;
                obj.LocalForces=s.LocalForces;
                obj=obj.Init(s);
            end
        end
    end
    
%%% PRIVATE METHODS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    methods (Access=private)                
        %Generation of Permutation Matrix, given a vector that states controlled
        %coordinates
        obj = GeneratePermutMatrix(obj)
        %INPUT: DoF-Dimensional vector of zeros and ones
        %OUTPUT: Permutation Matrix that allows for ones to be grouped in the first
        %elements of the vector and move other elements at the end.
        
        %       Initialize PermutMatrix (To be expanded)
        obj = Init(obj,s)   

    end   

end

