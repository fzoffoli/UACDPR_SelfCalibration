function obj=ComputeHDRPY(obj)
%%Compute Matrixes H & D from
s1=sin(obj.Pose(4));
s2=sin(obj.Pose(5));
c1=cos(obj.Pose(4));
c2=cos(obj.Pose(5));

obj.H=[  0  -s1    c1*c2;
         0   c1    s1*c2;
         1   0    -s2];


obj.D=  [eye(3), zeros(3);
        zeros(3), obj.H];
    
    

end