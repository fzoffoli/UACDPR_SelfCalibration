function  F=find_equilibrium_fromtau(Pose,tau,Disturb,obj)

    Pose_P = obj.PermutMatrix*Pose;
    obj    = SetPosePAndUpdateGeom(obj,Pose_P);
    obj    = ComputeWrench(obj,Disturb);
    
     F=[obj.GeomJac.Cables*tau-obj.Wrench;]
end