function [out] = dRy(th)
%Derivative wrt th of elementary y-axis rotation matrix
out=[
    -sin(th)    0    cos(th);
    0           0     0;
    -cos(th)    0   -sin(th);
    ];
end
