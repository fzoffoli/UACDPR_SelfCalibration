% This Method Computes the ortogonal and parallel matrix of the analitic
% permuted jacobian and of the geometric jacobian.
% The computed Jacobians can be found in:
% 
% UACDPR.AnalJac_Port ---> Ortogonal Permuted Analitic Jacobian
% UACDPR.GeomJac_Port ---> Ortogonal Permuted Geometric Jacobian
% 
% UACDPR.GeomJacl_ort ---> Ortogonal Geometric Jacobian
% UACDPR.GeomJacl_par ---> Parallel Geometric Jacobian (Pseudo inverse)
% 
% UACDPR.AnalJac_Ppar ---> Parallel Permuted Analitic Jacobian (Pseudo inverse)
% UACDPR.GeomJac_Ppar ---> Parallel Permuted Geometric Jacobian (Pseudo inverse)


function UACDPR = ComputeJaclOrtPar_modified(UACDPR)% modified to work for a 5cable cdpr as a 4 cable cdpr

J_d=UACDPR.AnalJac.Cables_P(1:4,1:4);
J_f=UACDPR.AnalJac.Cables_P(1:4,4+1:end);

% UACDPR.AnalJac.Cables_Port=[-inv(J_d')*J_f';
UACDPR.AnalJac.Cables_Port=[-linsolve(J_d,J_f);
                    eye(length(UACDPR.DependencyVect)-4)];



UACDPR.AnalJac.Cables_Ppar=[inv(J_d);
                    zeros(length(UACDPR.DependencyVect)-4,4)];
                
                


%%% The Model described in PhD Edoardo Idà thesis uses the following representation
%%% of the geometric ortogonal and parallel Jacobians. Nevertheless this
%%% representation brings with its use problems due to singularity of the
%%% space parametrization chosen (e.g. if Tilt and torsion is used, when tilt angle is null the analitic jacobian retains rank 3,although the geometric jacobian computed this way will have rank 2 due to D not having full rank.)
%%% As such, this computation will not be used and, instead, a direct
%%% computation of the geometric Jacobian is used.
% 
% UACDPR.GeomJac.Cables_ort = UACDPR.EndEffector.D*UACDPR.PermutMatrix'*UACDPR.AnalJac.Cables_Port;
% UACDPR.GeomJac.Cables_par = UACDPR.EndEffector.D*UACDPR.PermutMatrix'*UACDPR.AnalJac.Cables_Ppar;
% 
%%% The Direct computation is 

J_d=UACDPR.GeomJac.Cables_P(1:4,1:4);
J_f=UACDPR.GeomJac.Cables_P(1:4,(4+1):end);

% UACDPR.GeomJac.Cables_Port=[-inv(J_d')*J_f';
UACDPR.GeomJac.Cables_Port=[-linsolve(J_d,J_f);
                    eye(length(UACDPR.DependencyVect)-4)];



UACDPR.GeomJac.Cables_Ppar=[inv(J_d);
                    zeros(length(UACDPR.DependencyVect)-4,4)];
                
UACDPR.GeomJac.Cables_ort=UACDPR.PermutMatrix'*UACDPR.GeomJac.Cables_Port;
UACDPR.GeomJac.Cables_par=UACDPR.PermutMatrix'*UACDPR.GeomJac.Cables_Ppar;
end

