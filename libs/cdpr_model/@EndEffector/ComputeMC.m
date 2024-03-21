%Computation of Matrix M and C

function obj= ComputeMC(obj,Twist)
        %INPUT: Platform, Twist and Rotation Matrix
        %OUTPUT: Inertia tensor about P, M and C Matrices as per PhD Edo Chapter 2.3
        
m = obj.Platform.Mass;
PtoG_f=obj.RotMatrix*obj.Platform.PtoG;
% PtoG_f=obj.Platform.PtoG;

obj.InertiaMatrix = obj.RotMatrix*(obj.Platform.BarycentricInertia)*obj.RotMatrix.'-m*skew(PtoG_f)*skew(PtoG_f);

obj.M = [m*eye(3)                         -m*skew(PtoG_f);
         m*skew(PtoG_f)         obj.InertiaMatrix;];
    
obj.C = [zeros(3)        -m*skew(Twist(4:6))*skew(PtoG_f);
         zeros(3)         skew(Twist(4:6))*obj.InertiaMatrix];

end