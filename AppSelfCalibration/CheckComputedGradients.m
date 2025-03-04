function [F,G] = CheckComputedGradients(MyUACDPR,zeta)

n_d = length(MyUACDPR.DependencyVect);
MyUACDPR = SetPoseAndUpdate0KIN(MyUACDPR,zeta);
MyUACDPR = ComputeJaclOrtPar(MyUACDPR);
MyUACDPR = ComputeJac_dpose(MyUACDPR);
MyUACDPR = ComputeGravityWrench(MyUACDPR);

%% gradient statics passed
% tau = ones(4,1)*44;
% F = -MyUACDPR.AnalJac.Cables'*tau+MyUACDPR.EndEffector.D'*MyUACDPR.Wrench;
% 
% GeomJac_dpose = permute(MyUACDPR.GeomJac_dpose.Cables,[2 3 1]);
% AnalJac_dpose = permute(MyUACDPR.AnalJac_dpose.Cables,[2 3 1]);
% D_dpose = permute(MyUACDPR.EndEffector.D_dpose,[2 3 1]);
% % vect = MyUACDPR.GeomJac.Cables'*tau;
% A = zeros(6,6);
% B = zeros(6,6);
% for i = 1:4
% A = A+AnalJac_dpose(:,:,i)*tau(i);
% end
% for j = 1:6
% B = B + D_dpose(:,:,j)*MyUACDPR.Wrench(j);
% end
% C = MyUACDPR.EndEffector.D'*[zeros(3,6); zeros(3) skew(MyUACDPR.Wrench(1:3))*skew(MyUACDPR.EndEffector.PtoG_global)]*MyUACDPR.EndEffector.D;
% 
% G = -A+B+C;

%% gradient parallel jacobian

foo_vector = 42.*ones(6,1);
F = MyUACDPR.AnalJac.Cables_Ppar'*foo_vector;

AnalJac_dpose = permute(MyUACDPR.AnalJac_dpose.Cables,[2 1 3]);
AnalJac_Pdpose_c = AnalJac_dpose(:,:,logical(MyUACDPR.DependencyVect));
AnalJac_Pdpose_c = permute(AnalJac_Pdpose_c,[3 2 1]);
% AnalJac_Pdpose(:,:,MyUACDPR.CablesNumber+1:n_d) = AnalJac_dpose(:,:,~logical(MyUACDPR.DependencyVect));

AnalJac_Ppar_dpose = zeros(MyUACDPR.CablesNumber,n_d,n_d);
G_ = zeros(4,6);
for i = 1:n_d
    AnalJac_Ppar_dpose(1:4,1:4,i) = -MyUACDPR.AnalJac.Cables_Ppar(1:4,1:4)'*AnalJac_Pdpose_c(:,:,i)*MyUACDPR.AnalJac.Cables_Ppar(1:4,1:4)';
    AnalJac_Ppar_dpose(1:4,5:6,i) = zeros(4,2);
    G_(:,i) = AnalJac_Ppar_dpose(:,:,i)*foo_vector;
end
G = G_;
end