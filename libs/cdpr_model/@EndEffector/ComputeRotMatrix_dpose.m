function obj = ComputeRotMatrix_dpose(obj)
%ROTMATRIX_DPOSECOMP Rotation matrix derivative wrt pose
    OrientType = obj.OrientType;
    Pose = obj.Pose;
    
    R_dpose = zeros(3,3,6);    
    if OrientType=="Edo"
        % UACDPR.RotMatrix=Rz(UACDPR.Pose(4))*Ry(UACDPR.Pose(5))*Rz(UACDPR.Pose(6)-UACDPR.Pose(4));
        R_dpose(:,:,4) = dRz(Pose(4))*Ry(Pose(5))*Rz(Pose(6)-Pose(4)) - ...
                        Rz(Pose(4))*Ry(Pose(5))*dRz(Pose(6)-Pose(4));
        R_dpose(:,:,5) = Rz(Pose(4))*dRy(Pose(5))*Rz(Pose(6)-Pose(4));
        R_dpose(:,:,6) = Rz(Pose(4))*Ry(Pose(5))*dRz(Pose(6)-Pose(4));
    
    elseif OrientType=="TaitBryan"
        % UACDPR.RotMatrix=Rx(UACDPR.Pose(4))*Ry(UACDPR.Pose(5))*Rz(UACDPR.Pose(6));
        R_dpose(:,:,4) = dRx(Pose(4))*Ry(Pose(5))*Rz(Pose(6));
        R_dpose(:,:,5) = Rx(Pose(4))*dRy(Pose(5))*Rz(Pose(6));
        R_dpose(:,:,6) = Rx(Pose(4))*Ry(Pose(5))*dRz(Pose(6));
    else
        % UACDPR.RotMatrix=Rz(UACDPR.Pose(4))*Ry(UACDPR.Pose(5))*Rx(UACDPR.Pose(6));
        R_dpose(:,:,4) = dRz(Pose(4))*Ry(Pose(5))*Rx(Pose(6));
        R_dpose(:,:,5) = Rz(Pose(4))*dRy(Pose(5))*Rx(Pose(6));
        R_dpose(:,:,6) = Rz(Pose(4))*Ry(Pose(5))*dRx(Pose(6));
    end

    obj.RotMatrix_dpose = R_dpose;

end

