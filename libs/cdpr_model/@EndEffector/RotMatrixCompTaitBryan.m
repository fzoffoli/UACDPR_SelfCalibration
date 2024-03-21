function obj = RotMatrixCompTaitBryan(obj)  % = YPR
    %Computation of the rotational matrix
    obj.RotMatrix=Rx(obj.Pose(4))*Ry(obj.Pose(5))*Rz(obj.Pose(6));

end
        
