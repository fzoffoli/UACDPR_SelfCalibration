%Computation of Controlled motion stiffness and free motion stiffness as,
%respectively, ortogonal of active stiffness matrix and pseudo inverse of
%active stiffness matrix
% Results can be found in:
% 
% UACDPR.FMS ---> Free Motion Stiffness
% UACDPR.CMS ---> Controlled Motion Stiffness

function UACDPR = ComputeCMSFMS(UACDPR)

% Support variables
P=UACDPR.PermutMatrix;
K=UACDPR.ActiveStiffness;
E=UACDPR.MatrixE;
F=UACDPR.MatrixF;
D=UACDPR.EndEffector.D;
J_par=P'*UACDPR.AnalJac.Cables_Ppar;
J_ort = P'*UACDPR.AnalJac.Cables_Port;

% Computation
UACDPR.FMS=UACDPR.GeomJac.Cables_ort'*((K+E)*D-F)*J_ort;
% Kd_ort = UACDPR.GeomJac.Cables_ort'*(K+E)*UACDPR.GeomJac.Cables_ort;
% Kd_ort=UACDPR.Cables.GeomJacl_ort'*((K+E)*D-F)*J_par;
% UACDPR.CMS=-UACDPR.GeomJacl_par'*((K+E)*D-F)*(-J_ort*inv(UACDPR.FMS)*Kd_ort+J_par);

end

