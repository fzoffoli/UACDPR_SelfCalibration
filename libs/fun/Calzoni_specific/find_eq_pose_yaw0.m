function F = find_eq_pose_yaw0(Pose_in,yaw,Pose_obj,Disturb,length5,obj)

    Pose=[Pose_in;yaw];
    Pose_P = obj.PermutMatrix*Pose;
    obj    = SetPosePAndUpdateGeom(obj,Pose_P);
    obj    = ComputeWrench(obj,Disturb);
    
%     obj    = ComputeJaclOrtPar(obj);
%     t_5    = obj.Trasmission.Pulley{5}.CableFrame{1};
%     localpoints = obj.EndEffector.LocalAttachPoints_;
    temp=obj.GeomJac.Cables;
%     temp2=pinv(obj.GeomJac.Cables)*obj.Wrench;
%     temp3=obj.GeomJac.Cables*temp2-obj.Wrench;
  cab_leng= obj.CableLengths_;              
                   
    F=[
        obj.EndEffector.GlobalAttachPoints{5}-Pose_obj;
%         cab_leng(5)-length5;

%         obj.GeomJac.Cables_ort'*(obj.Wrench)
%         null(obj.GeomJac.Cables.').'*obj.Wrench;
null(temp(1:6,1:4).').'*obj.Wrench;
        ];
end