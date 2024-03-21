function R = RotMatrixYPR(vect)
    %Computation of the rotational matrix
    R = Rz(vect(1))*Ry(vect(2))*Rx(vect(3));

end
        
