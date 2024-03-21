function out= findtau_frompose_equilibrium(Pose,Disturb,obj)

obj    = SetPosePAndUpdateGeom(obj,obj.PermutMatrix*Pose);
obj    = ComputeWrench(obj,Disturb);
% obj    = ComputeJaclOrtPar(obj);
% out    = obj.GeomJac.Cables_par.'*obj.Wrench;
out    = pinv(obj.GeomJac.Cables)*obj.Wrench;
end