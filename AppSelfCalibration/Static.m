function [F]=Static(pose,obj,Disturb,tau)
% function [F,Jac]=Static(obj,disturb,tau)
    obj=ComputeWrench(obj,Disturb);
    obj=SetPoseAndUpdate0KIN(obj,pose);
    obj = ComputeJaclOrtPar(obj);
    
    %Set Function Objective;        
    F=obj.GeomJac.Cables.'*tau-obj.Wrench; %pag 28 Phd, (2.82) pongo questa uguale a 0 
            
end