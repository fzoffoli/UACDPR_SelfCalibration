% This method computes parallel matrixes of the equivalent
% mass and damping permuted matrix as per PhD Edo chapter 2.3.2

function [UACDPR] = ComputeMC_Ppar(UACDPR)

UACDPR.M_Ppar=-UACDPR.GeomJac.Cables_par'*UACDPR.EndEffector.M*UACDPR.EndEffector.D*UACDPR.PermutMatrix';
UACDPR.C_Ppar=-UACDPR.GeomJac.Cables_par'*(UACDPR.EndEffector.M*UACDPR.EndEffector.D_dt+UACDPR.EndEffector.C*UACDPR.EndEffector.D)*UACDPR.PermutMatrix';

end

