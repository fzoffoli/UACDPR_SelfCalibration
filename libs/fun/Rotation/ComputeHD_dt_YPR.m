function [H_dt,D_dt] = ComputeHD_dt_YPR(angles,angles_dt)
%%Compute Matrixes H & D from
s1=sin(angles(1));
s2=sin(angles(2));
c1=cos(angles(1));
c2=cos(angles(2));


mat=zeros(3);
mat(1,2) = -c1*angles_dt(1);
mat(1,3) = -s1*c2*angles_dt(1) - c1*s2*angles_dt(2);
mat(2,2) = -s1*angles_dt(1);
mat(2,3) = c1*c2*angles_dt(1) - s1*s2*angles_dt(2);
mat(3,3) = -c2*angles_dt(2);

H_dt=mat;
D_dt=[zeros(3,6);zeros(3),H_dt];
    
    
end

