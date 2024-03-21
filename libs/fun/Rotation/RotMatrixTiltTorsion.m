function R = RotMatrixTiltTorsion(vect)

    %Computation of the rotational matrix R = Rz(vect(1))*Ry(vect(2))*Rz(vect(3)-vect(1))

    R = Rz(vect(1))*Ry(vect(2))*Rz(vect(3)-vect(1));
end
        
