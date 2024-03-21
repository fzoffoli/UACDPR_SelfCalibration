%This Method allows for the positioning of the robot with a specific pose,
%which is to be given as input in its permuted form.
% Some of the computed variables are:
% 
% -UACDPR.EndEffector.RotMatrix//UACDPR.RotMatrix_ ---> Rotation Matrix
% -UACDPR.EndEffector.GlobalAttachPoints//UACDPR.GlobalAttachPoints_ ---> Global Attach Points
% -UACDPR.EndEffector.D//UACDPR.D_ ---> D Matrix
% -UACDPR.EndEffector.H//UACDPR.H_ ---> H Matrix
% -UACDPR.Trasmission.Pulley{i}.TangencyAngle ---> Cable-Pulley tangency angle
% -UACDPR.Trasmission.Pulley{i}.SwivelAngle ---> Pulley Swivel angle
% -UACDPR.Trasmission.Pulley{i}.PulleyFrame ---> Pulley referenced frames
% -UACDPR.Trasmission.Pulley{i}.CableFrame ---> Cable referenced frames
% -UACDPR.Trasmission.CableLengths//UACDPR.CableLengths_ ---> Cable length vector
% -UACDPR.Trasmission.Cable{i}.CableVector --->pulley-exit to platform attach
% point vector
% -UACDPR.FirstOrderKin.GeomJacl//UACDPR.GeomJacl_ ---> Geometric Jacobian
% -UACDPR.FirstOrderKin.AnalJacl//UACDPR.AnalJacl_ ---> Analitic Jacobian
% -UACDPR.GeomJacl_P ---> Permuted Geometric Jacobian
% -UACDPR.AnalJacl_P ---> Permuted Analitic Jacobian
% 
% 

function UACDPR = SetPoseAndUpdate0KIN(UACDPR,Pose)

%Call for End Effector Update
UACDPR.EndEffector = UpdateEndEffector0KIN(UACDPR.EndEffector,Pose);

%Call for Trasmission Update
UACDPR.Trasmission = UpdateTrasmission(UACDPR.Trasmission,UACDPR.EndEffector.GlobalAttachPoints);

%Call for Cable Jacobian Update
LocalAttachPoints=UACDPR.EndEffector.GlobalAttachPoints;

for i=1:length(UACDPR.EndEffector.GlobalAttachPoints)
    LocalAttachPoints{i}=UACDPR.EndEffector.GlobalAttachPoints{i}-UACDPR.EndEffector.Pose(1:3); 
end

UACDPR = UpdateJacl(UACDPR,LocalAttachPoints,UACDPR.Trasmission.Pulley,UACDPR.EndEffector.D);


UACDPR.AnalJac.Cables_P = UACDPR.AnalJac.Cables*UACDPR.PermutMatrix.';

UACDPR.GeomJac.Cables_P = UACDPR.GeomJac.Cables*UACDPR.PermutMatrix.';

UACDPR = ComputeAngleJac(UACDPR,UACDPR.EndEffector.LocalAttachPoints_,UACDPR.Trasmission.Cable,UACDPR.Trasmission.Pulley,UACDPR.EndEffector.D);

end

