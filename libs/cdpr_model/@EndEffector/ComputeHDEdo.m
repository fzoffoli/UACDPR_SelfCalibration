function obj=ComputeHDEdo(obj)
%%Compute Matrixes H & D from Edo PhD Chapter 2.2
s1=sin(obj.Pose(4));
s2=sin(obj.Pose(5));
c1=cos(obj.Pose(4));
c2=cos(obj.Pose(5));

obj.H=[-c1*s2   	-s1      c1*s2;
        -s1*s2      c1      s1*s2;
        1-c2        0       c2];


obj.D=  [eye(3), zeros(3);
        zeros(3), obj.H];
    
    

end