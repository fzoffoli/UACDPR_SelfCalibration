function rotmatrix = RotMatrixRPY(vect)
    %Computation of the rotational matrix
    rotmatrix=Rz(vect(3))*Ry(vect(2))*Rx(vect(1));

end