function obj = RotMatrixCompEdo(obj)

    %Computation of the rotational matrix

    obj.RotMatrix=Rz(obj.Pose(4))*Ry(obj.Pose(5))*Rz(obj.Pose(6)-obj.Pose(4));
end
        
