function [H,D,H_dt,D_dt] = ComputeHD_HDdt_caricosvinc(input)
%%Compute Matrixes H & D from
s1=sin(input.pose(4));
s2=sin(input.pose(5));
c1=cos(input.pose(4));
c2=cos(input.pose(5));


mat = eye(3);
mat(1,3) = s2;
mat(2,2) = c1;
mat(2,3) = -s1*c2;
mat(3,2) = s1;
mat(3,3) = c1*c2;

H=mat;


D=  [eye(3), zeros(3);
        zeros(3), H];
    

mat = zeros(3);
mat(1,3) = c2*input.pose_dt(5);
mat(2,2) = -s1*input.pose_dt(4);
mat(2,3) = -c1*c2*input.pose_dt(4)+s1*s2*input.pose_dt(5);
mat(3,2) = c1*input.pose_dt(4);
mat(3,3) = -s1*c2*input.pose_dt(4)-c1*s2*input.pose_dt(5);

H_dt=mat;
D_dt=[zeros(3,6);zeros(3),H_dt];
    
    
    
    
end

