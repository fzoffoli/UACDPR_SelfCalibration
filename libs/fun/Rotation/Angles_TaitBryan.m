function t = Angles_TaitBryan(R)

%TAITBRYAN_ANGLES gives angles: t1,t2,t3 given the rotation matrix: R = Rx(t1)Ry(t2)Rz(t3)

% !!!!!!!!!!!!!!!!! NOT SURE IT WORKS !!!!!!!!!!!!!!!!!!!!!
    t1 = atan2(-R(2,3),R(3,3));
    t2 = atan2(R(1,3),R(3,3)/cos(t1));
    t3 = atan2(-R(1,2),R(1,1));
    t = [t1; t2; t3];
end

