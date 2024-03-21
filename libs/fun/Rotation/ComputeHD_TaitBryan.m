function [H,D] = ComputeHD_TaitBryan(angles)
%%Compute Matrixes H & D from
s1=sin(angles(1));
s2=sin(angles(2));
c1=cos(angles(1));
c2=cos(angles(2));


mat = eye(3);
mat(1,3) = s2;
mat(2,2) = c1;
mat(2,3) = -s1*c2;
mat(3,2) = s1;
mat(3,3) = c1*c2;

H=mat;


D=  [eye(3), zeros(3);
        zeros(3), H];  
    
    
end

