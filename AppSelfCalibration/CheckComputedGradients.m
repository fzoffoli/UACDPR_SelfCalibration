function [F,G] = CheckComputedGradients(MyUACDPR,zeta)

MyUACDPR = SetPoseAndUpdate0KIN(MyUACDPR,zeta);
MyUACDPR = ComputeJac_dpose(MyUACDPR);
MyUACDPR = ComputeGravityWrench(MyUACDPR);

tau = ones(4,1)*44;
F = -MyUACDPR.AnalJac.Cables'*tau+MyUACDPR.EndEffector.D'*MyUACDPR.Wrench;

GeomJac_dpose = permute(MyUACDPR.GeomJac_dpose.Cables,[2 3 1]);
AnalJac_dpose = permute(MyUACDPR.AnalJac_dpose.Cables,[2 3 1]);
D_dpose = permute(MyUACDPR.EndEffector.D_dpose,[2 3 1]);
% vect = MyUACDPR.GeomJac.Cables'*tau;
A = zeros(6,6);
B = zeros(6,6);
for i = 1:4
A = A+AnalJac_dpose(:,:,i)*tau(i);
end
for j = 1:6
B = B + D_dpose(:,:,j)*MyUACDPR.Wrench(j);
end
C = MyUACDPR.EndEffector.D'*[zeros(3,6); zeros(3) skew(MyUACDPR.Wrench(1:3))*skew(MyUACDPR.EndEffector.PtoG_global)]*MyUACDPR.EndEffector.D;

G = -A+B+C;
end