function t = Angles_RPY(R)

% Angles_RPY gives RPY angles: t1,t2,t3 given the rotation matrix: R = Rz(t3)Ry(t2)Rx(t1)
% (OneNote) Rz(alpha)Ry(beta)Rx(gamma)

    t1 = atan2(R(3,2),R(1,1));
    t2 = atan2(-R(3,1),sqrt(R(1,1)^2 + R(2,1)^2));
    t3 = atan2(R(2,1),R(1,1));
    t = [t1; t2; t3];
end

