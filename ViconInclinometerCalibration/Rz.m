function [out] = Rz(th)
%Elementary z-axis rotation matrix
out=[
    cos(th)    -sin(th)     0;
    sin(th)     cos(th)     0;
    0           0           1;
    ];
end

