%This method:
%-assigns first order pose time derivative;
%-launches Matrix H and D first time derivatives and equivalent mass and
%damping matrixes computations;

function EndEffector = UpdateEndEffector1KIN(EndEffector,Pose_dt,Twist)
EndEffector.Pose_dt = Pose_dt;

if EndEffector.OrientType=="Edo"
%     EndEffector = ComputeHDEdo_dt(EndEffector);
[H_dt,D_dt] = ComputeHD_dt_TiltTorsion(EndEffector.Pose(4:6),EndEffector.Pose_dt(4:6));

EndEffector.H_dt=H_dt;
EndEffector.D_dt=D_dt;
elseif EndEffector.OrientType=="TaitBryan"
%     EndEffector = ComputeHDTaitBryan_dt(EndEffector);
    [H_dt,D_dt] = ComputeHD_dt_TaitBryan(EndEffector.Pose(4:6),EndEffector.Pose_dt(4:6));
EndEffector.H_dt=H_dt;
EndEffector.D_dt=D_dt;
else
%     EndEffector = ComputeHDRPY_dt(EndEffector);
[H_dt,D_dt] = ComputeHD_dt_YPR(EndEffector.Pose(4:6),EndEffector.Pose_dt(4:6));
EndEffector.H_dt=H_dt;
EndEffector.D_dt=D_dt;

end

EndEffector = ComputeC(EndEffector,Twist);
end

