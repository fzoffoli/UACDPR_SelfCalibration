% This method calls for Trasmission
% Method to compute
% cable first and second order time derivatives. Makes use of Geometric
% jacobian, twist and their derivatives.

function UACDPR = ComputeAngleDerivatives(UACDPR)

UACDPR.Trasmission = UACDPR.Trasmission.ComputeAngleDerivatives(UACDPR.Twist,UACDPR.GeomJac);

end

