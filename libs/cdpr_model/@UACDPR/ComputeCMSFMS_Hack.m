%         Compute Controlled motion stiffness and free motion stiffness:
%         this version make use of the projection on the geometric jacobian
%         space to avoid parametrization singularity problems. As such it
%         works ONLY when no external forces are employed (Matrix F must be null)
% Results can be found in:
% 
% UACDPR.FMS ---> Free Motion Stiffness
% UACDPR.CMS ---> Controlled Motion Stiffness

function UACDPR = ComputeCMSFMS_Hack(UACDPR)

K = UACDPR.ActiveStiffness;
E = UACDPR.MatrixE;
% F=UACDPR.MatrixF;
% D=UACDPR.EndEffector.D;
% J_par=P'*UACDPR.AnalJac_Ppar;
% J_ort=P'*UACDPR.AnalJac_Port;

% Computation
UACDPR.FMS  = UACDPR.GeomJac.Cables_ort'*(K+E)*UACDPR.GeomJac.Cables_ort;
Kd_ort   = UACDPR.GeomJac.Cables_ort'*(K+E)*UACDPR.GeomJac.Cables_par;
UACDPR.CMS  = -UACDPR.GeomJac.Cables_par'*(K+E)*(-UACDPR.GeomJac.Cables_ort*linsolve(UACDPR.FMS,Kd_ort)+UACDPR.GeomJac.Cables_par);

end

