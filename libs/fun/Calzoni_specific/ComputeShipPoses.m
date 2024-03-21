function [Pose,Pose_dt,Pose_ddt]=ComputeShipPoses(A_ship,w_ship,phase_ship,Pose_robot,t)

Pose=zeros(6,1);
Pose_dt=zeros(6,1);
Pose_ddt=zeros(6,1);


for i=1:6
   Pose(i,1)=A_ship(i)*sin(w_ship(i)*t+phase_ship(i));
   Pose_dt(i,1)=w_ship(i)*A_ship(i)*cos(w_ship(i)*t+phase_ship(i));
   Pose_ddt(i,1)=-1*w_ship(i)^2*A_ship(i)*sin(w_ship(i)*t+phase_ship(i));
end

end