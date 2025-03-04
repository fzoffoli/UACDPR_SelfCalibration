function UACDPR = ComputeGravityWrench(UACDPR)


g = [0; 0; -UACDPR.EndEffector.Platform_Param.Mass*9.81];

UACDPR.Wrench = [g; skew(UACDPR.EndEffector.PtoG_global)*g];

end