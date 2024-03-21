function [H,D] = ComputeHD_TiltTorsion(angles)
%%Compute Matrixes H & D from
s1=sin(angles(1));
s2=sin(angles(2));
c1=cos(angles(1));
c2=cos(angles(2));

H=[-c1*s2   	-s1      c1*s2;
        -s1*s2      c1      s1*s2;
        1-c2        0       c2];


D=  [eye(3), zeros(3);
        zeros(3), H];
    
end

