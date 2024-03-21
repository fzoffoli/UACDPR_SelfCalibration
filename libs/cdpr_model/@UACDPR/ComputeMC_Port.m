function UACDPR = ComputeMC_Port(UACDPR)

UACDPR.M_Port=UACDPR.GeomJac.Cables_ort'*UACDPR.EndEffector.M*UACDPR.EndEffector.D*UACDPR.PermutMatrix';

UACDPR.C_Port=UACDPR.GeomJac.Cables_ort'*(UACDPR.EndEffector.M*UACDPR.EndEffector.D_dt+UACDPR.EndEffector.C*UACDPR.EndEffector.D)*UACDPR.PermutMatrix';

end

