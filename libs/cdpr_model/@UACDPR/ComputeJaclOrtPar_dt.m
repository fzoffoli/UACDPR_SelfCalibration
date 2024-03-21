% This method computes Cable Geometric and Analitic ortogonal and parallel
% matrixes in their 1st-order time derivative.
% Results can be found in:
% 
% -UACDPR.AnalJac_Port_dt ---> Ortogonal Permuted Analitic Cable Jacobian 1st-orde time derivative
% -UACDPR.AnalJac_Ppar_dt ---> Parallel Permuted Analitic Cable Jacobian 1st-orde time derivative (pseudo inverse)
% -UACDPR.GeomJac_ort_dt ---> Ortogonal Geometric Cable Jacobian 1st-orde time derivative
% -UACDPR.GeomJac_par_dt ---> Parallel Geometric Cable Jacobian 1st-orde time derivative (pseudo inverse)

function UACDPR = ComputeJaclOrtPar_dt(UACDPR)

J_d    = UACDPR.AnalJac.Cables_P(:,1:UACDPR.CablesNumber).';
J_d_dt = UACDPR.AnalJac.Cables_P_dt(:,1:UACDPR.CablesNumber).';

J_f    = UACDPR.AnalJac.Cables_P(:,UACDPR.CablesNumber+1:end).';
J_f_dt = UACDPR.AnalJac.Cables_P_dt(:,UACDPR.CablesNumber+1:end).';


UACDPR.AnalJac.Cables_Port_dt = [inv(J_d')*(J_d_dt'*inv(J_d')*J_f'-J_f_dt');
                        zeros(size(J_f,1))];


UACDPR.AnalJac.Cables_Ppar_dt = [-inv(J_d')*J_d_dt'*inv(J_d');
                        zeros(size(J_f,1),UACDPR.CablesNumber)];


%%% The Model described iin PhD Edoardo Idà thesis uses this representation
%%% of the geometric ortogonal and parallel Jacobians. Nevertheless this
%%% representation brings with its use problems due to singularity of the
%%% space parametrization chosen (e.g. if Tilt and torsion is used, when tilt angle is null the analitic jacobian retains rank 3,although the geometric jacobian computed this way will have rank 2 due to D not having full rank.)
%%% As such, this compoutation will not be used and, instead, a direct
%%% computation of the geometric Jacobian is used.
% 
% UACDPR.GeomJac.Cables_ort_dt = UACDPR.EndEffector.D_dt*UACDPR.PermutMatrix'*UACDPR.AnalJac.Cables_Port+UACDPR.EndEffector.D*UACDPR.PermutMatrix'*UACDPR.AnalJac.Cables_Port_dt;
% UACDPR.GeomJac.Cables_par_dt = UACDPR.EndEffector.D_dt*UACDPR.PermutMatrix'*UACDPR.AnalJac.Cables_Ppar+UACDPR.EndEffector.D*UACDPR.PermutMatrix'*UACDPR.AnalJac.Cables_Ppar_dt;
% 
%%% The Direct computation is 
J_d    = UACDPR.GeomJac.Cables_P(:,1:UACDPR.CablesNumber).';
J_d_dt = UACDPR.GeomJac.Cables_P_dt(:,1:UACDPR.CablesNumber).';

J_f    = UACDPR.GeomJac.Cables_P(:,UACDPR.CablesNumber+1:end).';
J_f_dt = UACDPR.GeomJac.Cables_P_dt(:,UACDPR.CablesNumber+1:end).';


UACDPR.GeomJac.Cables_ort_dt = [inv(J_d')*(J_d_dt'*inv(J_d')*J_f'-J_f_dt');
                        zeros(size(J_f,1))];


UACDPR.GeomJac.Cables_par_dt = [-inv(J_d')*J_d_dt'*inv(J_d');
                        zeros(size(J_f,1),UACDPR.CablesNumber)];

end

