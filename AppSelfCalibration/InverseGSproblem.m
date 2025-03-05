function F = InverseGSproblem(MyUACDPR,pose_controlled,pose_free)
%INVERSEGSPROBLEM can be used as an handler function for a nonlinear
%solver to determine roll and pitch, for a given yaw and
%position (four cable UACDPR).

pose = MyUACDPR.PermutMatrix'*[pose_controlled;pose_free];
MyUACDPR = SetPoseAndUpdate0KIN(MyUACDPR,pose);
MyUACDPR = ComputeJaclOrtPar(MyUACDPR);
MyUACDPR = ComputeGravityWrench(MyUACDPR);

F = MyUACDPR.GeomJac.Cables_ort'*MyUACDPR.Wrench;
end