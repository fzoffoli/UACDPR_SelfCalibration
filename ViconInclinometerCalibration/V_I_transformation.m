% This function represents the transformation between the inclinometer
% (semi)absolute frame and the vicon imposed fixed frame through the
% relation R_v = R_vi*R_i*R_c,

function F = V_I_transformation(x,eps_vicon,eps_inclinometer,phi_0)

R_z0 = Rz(phi_0);
R_c = RotMatrixRPY(x(1:3));
R_vi = RotMatrixRPY(x(4:6));
F=zeros(size(eps_vicon));
for i = 1:length(eps_vicon)
    R_i = R_z0'*RotMatrixRPY(eps_inclinometer(:,i));
    eps_vicon_est = Angles_RPY(R_vi*R_i*R_c);
    F(:,i) = eps_vicon(:,i)-eps_vicon_est;
end

F=reshape(F,[length(F)*3 1]);
end