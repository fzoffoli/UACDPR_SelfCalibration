function obj=ComputeHDTaitBryan(obj)
%%Compute Matrixes H & D from
s1=sin(obj.Pose(4));
s2=sin(obj.Pose(5));
c1=cos(obj.Pose(4));
c2=cos(obj.Pose(5));


mat = eye(3);
mat(1,3) = s2;
mat(2,2) = c1;
mat(2,3) = -s1*c2;
mat(3,2) = s1;
mat(3,3) = c1*c2;

obj.H=mat;


obj.D=  [eye(3), zeros(3);
        zeros(3), obj.H];
    
    

end