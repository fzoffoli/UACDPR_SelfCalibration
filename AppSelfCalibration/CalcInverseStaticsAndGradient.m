function [F,G] = CalcInverseStaticsAndGradient(MyUACDPR,zeta)
%CALCINVERSESTATICSANDGRADIENT computes the inverse statics and its
%gradient when parallel jacobian is used

n_d = length(MyUACDPR.DependencyVect);
MyUACDPR = SetPoseAndUpdate0KIN(MyUACDPR,zeta);
MyUACDPR = ComputeJaclOrtPar(MyUACDPR);
MyUACDPR = ComputeJac_dpose(MyUACDPR);
MyUACDPR = ComputeGravityWrench(MyUACDPR);

% cost function
F = MyUACDPR.AnalJac.Cables_Ppar'*MyUACDPR.PermutMatrix*MyUACDPR.EndEffector.D'*MyUACDPR.Wrench;

% gradient of the first term
A_vector = MyUACDPR.PermutMatrix*MyUACDPR.EndEffector.D'*MyUACDPR.Wrench;

AnalJac_dpose = permute(MyUACDPR.AnalJac_dpose.Cables,[2 1 3]);
AnalJac_Pdpose_c = AnalJac_dpose(:,:,logical(MyUACDPR.DependencyVect));
AnalJac_Pdpose_c = permute(AnalJac_Pdpose_c,[3 2 1]);

AnalJac_Ppar_dpose = zeros(MyUACDPR.CablesNumber,n_d,n_d);
G_a = zeros(MyUACDPR.CablesNumber,n_d);
for i = 1:n_d
    AnalJac_Ppar_dpose(1:4,1:4,i) = -MyUACDPR.AnalJac.Cables_Ppar(1:4,1:4)'*AnalJac_Pdpose_c(:,:,i)*MyUACDPR.AnalJac.Cables_Ppar(1:4,1:4)';
    AnalJac_Ppar_dpose(1:4,5:6,i) = zeros(4,2);
    G_a(:,i) = AnalJac_Ppar_dpose(:,:,i)*A_vector;
end

% gradient of the second and third terms
D_dpose = permute(MyUACDPR.EndEffector.D_dpose,[2 3 1]);

G_b = zeros(MyUACDPR.CablesNumber,n_d);
for j = 1:n_d
G_b = G_b + MyUACDPR.AnalJac.Cables_Ppar'*MyUACDPR.PermutMatrix*D_dpose(:,:,j)*MyUACDPR.Wrench(j);
end
G_c = MyUACDPR.AnalJac.Cables_Ppar'*MyUACDPR.PermutMatrix*MyUACDPR.EndEffector.D'*...
    [zeros(3,6); zeros(3) skew(MyUACDPR.Wrench(1:3))*skew(MyUACDPR.EndEffector.PtoG_global)]*MyUACDPR.EndEffector.D;

G = G_a + G_b + G_c;
end