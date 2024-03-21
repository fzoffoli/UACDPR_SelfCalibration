function obj = UpdateEndEffectorGeom(obj,Pose)
       %INPUT: Pose  [x,y,z,th,psi,phi]
       %OUTPUT: Set new pose, Update Rotational Matrix, Attach-points global coordinates and HD Matrixes
       
obj.Pose=Pose;
if obj.OrientType=="Edo"
obj = obj.RotMatrixCompEdo;

obj = GlobalComp(obj);

obj = obj.ComputeHDEdo;
elseif obj.OrientType=="TaitBryan"
obj = obj.RotMatrixCompTaitBryan;

obj = obj.GlobalComp;

obj = obj.ComputeHDTaitBryan;  
else
obj = obj.RotMatrixCompRPY;

obj = obj.GlobalComp;

obj = obj.ComputeHDRPY; 
end

end

