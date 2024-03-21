%This method has pose and its derivatives as input and updates first and second order kinematic
%variables. Some of the computed variables are:
% 
% -UACDPR.Twist ---> Twist
% -UACDPR.EndEffector.D_dt//UACDPR.D_dt_ ---> Matrix D 1st-order time derivative
% -UACDPR.EndEffector.H_dt//UACDPR.H_dt_ ---> Matrix H 1st-order time derivative
% -UACDPR.EndEffector.M//UACDPR.M_ ---> Equivalent Mass Matrix 
% -UACDPR.EndEffector.C//UACDPR.C_ ---> Equivalent Damping Matrix
% -UACDPR.EndEffector.Pose_dt//UACDPR.Pose_dt_ ---> Pose 1st-order time derivative
% 
% -UACDPR.FirstOrderKin.GeomJac_SA ---> Geometric Swivel-Angle Jacobian
% -UACDPR.FirstOrderKin.GeomJac_TA ---> Geometric Tangency-Angle Jacobian
% -UACDPR.FirstOrderKin.AnalJac_SA ---> Analitic Swivel-Angle Jacobian
% -UACDPR.FirstOrderKin.AnalJac_TA ---> Analitic Tangency-Angle Jacobian
% -UACDPR.GeomJacl_dt ---> Geometric cable Jacobian 1st-order time derivative
% -UACDPR.AnalJacl_dt ---> Analitic cable Jacobian 1st-order time derivative
% -UACDPR.GeomJac_P_dt ---> Permuted Geometric cable Jacobian 1st-order time derivative
% -UACDPR.AnalJac_P_dt ---> Permuted Analitic cable Jacobian 1st-order time derivative
% 
% If also Pose_ddt is specified:
% -UACDPR.Twist_dt ---> Twist 1st-order time derivative
% -UACDPR.EndEffector.Pose_ddt ---> Pose 2nd-order time derivative
% 
% Also, everything computed in UACDPR.SetPosePandUpdateGeom(PoseP):
% -UACDPR.EndEffector.RotMatrix//UACDPR.RotMatrix_ ---> Rotation Matrix
% -UACDPR.EndEffector.GlobalAttachPoints//UACDPR.GlobalAttachPoints_ ---> Global Attach Points
% -UACDPR.EndEffector.D//UACDPR.D_ ---> D Matrix
% -UACDPR.EndEffector.H//UACDPR.H_ ---> H Matrix
% -UACDPR.Trasmission.Pulley{i}.TangencyAngle ---> Cable-Pulley tangency angle
% -UACDPR.Trasmission.Pulley{i}.SwivelAngle ---> Pulley Swivel angle
% -UACDPR.Trasmission.Pulley{i}.PulleyFrame ---> Pulley referenced frames
% -UACDPR.Trasmission.Pulley{i}.CableFrame ---> Cable referenced frames
% -UACDPR.Trasmission.CableLengths//UACDPR.CableLengths_ ---> Cable length vector
% -UACDPR.Trasmission.Cable{i}.CableVector --->pulley-exit-to-platform attach-point vector
% -UACDPR.FirstOrderKin.GeomJacl//UACDPR.GeomJacl_ ---> Geometric Jacobian
% -UACDPR.FirstOrderKin.AnalJacl//UACDPR.AnalJacl_ ---> Analitic Jacobian
% -UACDPR.GeomJacl_P ---> Permuted Geometric Jacobian
% -UACDPR.AnalJacl_P ---> Permuted Analitic Jacobian

function UACDPR = SetPoseAndUpdate1KIN(UACDPR,Pose,Pose_dt,Pose_ddt)


    
    %Update pose twist and end effector  
    UACDPR.Twist       = UACDPR.EndEffector.D*Pose_dt;

    UACDPR.EndEffector    = UpdateEndEffector1KIN(UACDPR.EndEffector,Pose_dt,UACDPR.Twist);
    
    %Compute Jacobians
    
%     UACDPR.Trasmission = ComputeAngleDerivatives(UACDPR.Trasmission,UACDPR.Twist,UACDPR.GeomJac);
    
    UACDPR = UpdateJacl_dt(UACDPR,UACDPR.Twist,UACDPR.EndEffector.LocalAttachPoints_,UACDPR.GeomJac,...
                                                         UACDPR.Trasmission.Pulley,UACDPR.Trasmission.Cable,UACDPR.EndEffector.D,UACDPR.EndEffector.D_dt);


    UACDPR.AnalJac.Cables_P_dt   = UACDPR.AnalJac.Cables_dt*UACDPR.PermutMatrix.';
    UACDPR.GeomJac.Cables_P_dt = UACDPR.GeomJac.Cables_dt*UACDPR.PermutMatrix.';
    
    UACDPR = ComputeAngleJac_dt(UACDPR,UACDPR.EndEffector.LocalAttachPoints_,UACDPR.Trasmission.Cable,UACDPR.Trasmission.Pulley,UACDPR.EndEffector.D);

    if nargin==3 %IF only twist is given don't update second order kinematic
    else % Update first and second order kinematic
    UACDPR.Twist_dt    = UACDPR.EndEffector.D*Pose_ddt+UACDPR.EndEffector.D_dt*Pose_dt;
    
    UACDPR.EndEffector = UpdateEndEffector2KIN(UACDPR.EndEffector,Pose_ddt);
    end

