%Computation of Matrix M and C

function obj= ComputeM(obj)
        %INPUT: Platform, Twist and Rotation Matrix
        %OUTPUT: Inertia tensor about P, M and C Matrices as per PhD Edo Chapter 2.3
        
m = obj.Platform_Param.Mass;
obj.PtoG_global=obj.RotMatrix*obj.Platform_Param.PtoG;
% PtoG_f=obj.Platform.PtoG;

obj.InertiaMatrix = obj.RotMatrix*(obj.Platform_Param.BarycentricInertia)*obj.RotMatrix.'-m*skew(obj.PtoG_global)*skew(obj.PtoG_global);

obj.M = [m*eye(3)                         -m*skew(obj.PtoG_global);
         m*skew(obj.PtoG_global)         obj.InertiaMatrix;];
    
% obj.C = [zeros(3)        -m*skew(Twist(4:6))*skew(PtoG_f);
%          zeros(3)         skew(Twist(4:6))*obj.InertiaMatrix];

end