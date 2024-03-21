function obj = RotMatrixCompRPY(obj)    %SBAGLIATO! NON USARE MAI
    %Computation of the rotational matrix
    obj.RotMatrix=Rz(obj.Pose(4))*Ry(obj.Pose(5))*Rx(obj.Pose(6));

end
        
