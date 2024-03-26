% Zero order direct kinematics
% INPUT cable lengths, free coordinates
% OUTPUT controlled coordinates
function F = SetControlledPose0KIN(MyUACDPR,zeta_c,zeta_f,l)

pose = [zeta_c;zeta_f];
MyUACDPR = SetPoseAndUpdate0KIN(MyUACDPR,pose);
F = 0;

end