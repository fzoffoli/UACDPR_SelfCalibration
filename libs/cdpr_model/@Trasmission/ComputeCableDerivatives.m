% This method makes use of jacobians and twist/twist derivative to compute
% cable first and second order time derivatives.

function Trasmission = ComputeCableDerivatives(Trasmission,GeomJacl,GeomJacl_dt,Twist,Twist_dt)

    Trasmission.CableLengths_dt=GeomJacl.'*Twist;
    Trasmission.CableLengths_ddt=GeomJacl.'*Twist_dt+GeomJacl_dt.'*Twist;

end

