function [H,D] = ComputeHD_YPR(angles)
%%Compute Matrixes H & D from
s1=sin(angles(1));
s2=sin(angles(2));
c1=cos(angles(1));
c2=cos(angles(2));

H=[  0  -s1    c1*c2;
         0   c1    s1*c2;
         1   0    -s2];


D=  [eye(3), zeros(3);
        zeros(3), H];
    
    
end

