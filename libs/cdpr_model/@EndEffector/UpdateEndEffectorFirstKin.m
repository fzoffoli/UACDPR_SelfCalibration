%This method:
%-assigns first order pose time derivative;
%-launches Matrix H and D first time derivatives and equivalent mass and
%damping matrixes computations;

function obj = UpdateEndEffectorFirstKin(obj,Pose_dt,Twist)
obj.Pose_dt = Pose_dt;
if obj.OrientType=="Edo"
obj = ComputeHDEdo_dt(obj);
elseif obj.OrientType=="TaitBryan"
obj = ComputeHDTaitBryan_dt(obj);
else
obj = ComputeHDRPY_dt(obj);
end
obj = ComputeMC(obj,Twist);
end

