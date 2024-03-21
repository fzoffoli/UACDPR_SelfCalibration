function [F,Jac]=GeoStatic_modified(pose_f,obj,Disturb,pose_d)
	x=0.25;
    y=0;
    z=-6;
    Yaw=0;
    Roll=pose_f(1);
    Pitch=pose_f(2);
    
    obj=SetPoseAndUpdate0KIN(obj,[x;y;z;Roll;Pitch;Yaw]);
    obj=ComputeWrench(obj,Disturb);
    
    obj = ComputeJaclOrtPar_modified(obj);
    
    %Set Function Objective;        
    F=obj.GeomJac.Cables_ort'*(obj.Wrench-obj.GeomJac.Cables(5,:).'*100);
%      
%     obj=obj.SetStaticCableTensions;
%     obj=obj.ComputeActiveStiffness;  
% 
%     fullJac=-obj.GeomJac.Cables_ort'*(obj.ActiveStiffness+obj.MatrixE)*obj.EndEffector.D*obj.PermutMatrix';
%     Jac=fullJac(:,obj.CablesNumber+1:end);
%          
Jac=0;
end
