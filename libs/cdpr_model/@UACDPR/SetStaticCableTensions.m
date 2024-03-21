% Compute Cable tension for static conditions
% Cable tensions can be found in:
% 
% UACDPR.CableTensions_
% UACDPR.Trasmission.CableTensions

function UACDPR = SetStaticCableTensions(UACDPR)

UACDPR.Trasmission.CableTensions=UACDPR.GeomJac.Cables_par'*UACDPR.Wrench; % pag 28 phd, (2.84)

end

