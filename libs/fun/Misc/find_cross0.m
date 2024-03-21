function F=find_cross0(UACDPR,Pose_fix,Pose_unfix,Pose_frame)


    UACDPR = CorrectGeometry(UACDPR,Pose_frame);
    UACDPR=SetPoseAndUpdate0KIN(UACDPR,[Pose_fix(1:end-1);Pose_unfix;Pose_fix(end)]);

    F=cross(UACDPR.Trasmission.Cable{5}.CableVector,UACDPR.EndEffector.PtoG_global)

end