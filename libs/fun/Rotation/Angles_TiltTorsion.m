function t = Angles_TiltTorsion(R)

%TILTTORSION_ANGLES gives angles: t1,t2,t3 given the rotation matrix: R = Rz(t1)Ry(t2)Rz(t3-t1)
    t1 = atan2(R(2,3),R(1,3));
    t2 = atan2(R(1,3)/cos(t1),R(3,3));
    t3 = atan2(R(3,2),-R(3,1)) + t1;
    t = [t1; t2; t3];
end

