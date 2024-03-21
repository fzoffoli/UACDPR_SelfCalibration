function [out] = Ry(th)
%Elementary y-axis rotation matrix
out=[
    cos(th)     0   sin(th);
    0           1     0;
    -sin(th)    0   cos(th);
    ];
end
