function [H_dt,D_dt] = ComputeHD_dt_TiltTorsion(angles,angles_dt)
%%Compute Matrixes H & D from
s1=sin(angles(1));
s2=sin(angles(2));
c1=cos(angles(1));
c2=cos(angles(2));
dt1=angles_dt(1);
dt2=angles_dt(2);


% H_dp{1}=[0  -c1   -s1*c2;
%          0  -s1    c1*c2;
%          0   0     0];
%      
% H_dp{2}=[0 0 -c1*s2;
%          0 0 -s1*s2;
%          0 0 -c2];
%      
% H_dp{3}=zeros(3);
% 
% obj.H_dt=zeros(3);
% 
% for i=1:length(obj.Orient)
%     obj.H_dt=H_dp{i}*obj.Pose_dt(3+i)+obj.H_dt;
% end

mat=zeros(3);
mat(1,1) = s1*dt1*s2-c1*c2*dt2;
mat(1,2) = -c1*dt1;
mat(1,3) = -s1*dt1*s2+c1*c2*dt2;
mat(2,1) = -c1*dt1*s2-s1*c2*dt2;
mat(2,2) = -s1*dt1;
mat(2,3) = c1*dt1*s2+s1*c2*dt2;
mat(3,1) = s2*dt2;
mat(3,3) = -s2*dt2;

H_dt=mat;
D_dt=[zeros(3,6);zeros(3),H_dt];

    
end

