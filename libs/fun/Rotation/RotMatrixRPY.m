function R = RotMatrixRPY(vect)
    % Computation of the rotational matrix R = Rz(vect(3))*Ry(vect(2))*Rx(vect(1))
    % (OneNote) Rz(alpha)Ry(beta)Rx(gamma)

    R = Rz(vect(3))*Ry(vect(2))*Rx(vect(1));

end
        
