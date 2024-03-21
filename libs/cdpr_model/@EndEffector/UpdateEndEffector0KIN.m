function EndEffector = UpdateEndEffector0KIN(EndEffector,Pose)
       %INPUT: Pose  [x,y,z,th,psi,phi]
       %OUTPUT: Set new pose, Update Rotational Matrix, Attach-points global coordinates and HD Matrixes
       
EndEffector.Pose=Pose;

if EndEffector.OrientType=="Edo"
EndEffector.RotMatrix = RotMatrixTiltTorsion(EndEffector.Pose(4:6));

EndEffector = GlobalComp(EndEffector);

[H,D]=ComputeHD_TiltTorsion(EndEffector.Pose(4:6));
EndEffector.H=H;
EndEffector.D=D;

elseif EndEffector.OrientType=="TaitBryan"
EndEffector.RotMatrix = RotMatrixTaitBryan(EndEffector.Pose(4:6));

EndEffector = GlobalComp(EndEffector);

[H,D] = ComputeHD_TaitBryan(EndEffector.Pose(4:6));
EndEffector.H=H;
EndEffector.D=D;
    
else
EndEffector.RotMatrix = RotMatrixYPR(EndEffector.Pose(4:6));

EndEffector = GlobalComp(EndEffector);

[H,D] = ComputeHD_YPR(EndEffector.Pose(4:6));
EndEffector.H=H;
EndEffector.D=D;    
end


EndEffector = ComputeM(EndEffector);
end

