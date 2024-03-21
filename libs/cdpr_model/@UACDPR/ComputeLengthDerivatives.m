% This method calls for Trasmission
% Method to compute
% cable first and second order time derivatives. Makes use of Geometric
% jacobian, twist and their derivatives.

function UACDPR = ComputeLengthDerivatives(UACDPR)

UACDPR.Trasmission = UACDPR.Trasmission.ComputeCableDerivatives(UACDPR.GeomJac.Cables,UACDPR.GeomJac.Cables_dt,UACDPR.Twist,UACDPR.Twist_dt);

end

