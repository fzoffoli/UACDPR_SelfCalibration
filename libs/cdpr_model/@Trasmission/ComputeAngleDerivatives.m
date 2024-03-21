function Trasmission = ComputeAngleDerivatives(Trasmission,Twist,GeomJac)

Trasmission.Tangency_dt = GeomJac.Tangency*Twist;
Trasmission.Swivel_dt = GeomJac.Swivel*Twist;

end

