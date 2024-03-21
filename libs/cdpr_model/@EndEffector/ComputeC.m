function obj = ComputeC(obj,Twist)


obj.C = [zeros(3)        -obj.Platform_Param.Mass*skew(Twist(4:6))*skew(obj.PtoG_global);
         zeros(3)         skew(Twist(4:6))*obj.InertiaMatrix];

end

