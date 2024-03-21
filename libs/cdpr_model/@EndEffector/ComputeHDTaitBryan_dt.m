function obj = ComputeHDTaitBryan_dt(obj)

%Generate first order time derivative of matrixes H and D

s1=sin(obj.Pose(4));
s2=sin(obj.Pose(5));
c1=cos(obj.Pose(4));
c2=cos(obj.Pose(5));


% % H_dp{1}=[0  0   0;
% %          0  -s1    -c1*c2;
% %          0   c1     -s1*c2];
% %      
% % H_dp{2}=[0 0 c2;
% %          0 0 s1*s2;
% %          0 0 -c1*s2];
% %      
% % H_dp{3}=zeros(3);
% % 
% % obj.H_dt=zeros(3);
% % 
% % for i=1:3
% %     obj.H_dt=H_dp{i}*obj.Pose_dt(3+i)+obj.H_dt;
% % end

mat = zeros(3);
mat(1,3) = c2*obj.Pose_dt(5);
mat(2,2) = -s1*obj.Pose_dt(4);
mat(2,3) = -c1*c2*obj.Pose_dt(4)+s1*s2*obj.Pose_dt(5);
mat(3,2) = c1*obj.Pose_dt(4);
mat(3,3) = -s1*c2*obj.Pose_dt(4)-c1*s2*obj.Pose_dt(5);

obj.H_dt=mat;
obj.D_dt=[zeros(3,6);zeros(3),obj.H_dt];

end

