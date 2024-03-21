function [out] = dRz(th)
%Derivative wrt th of elementary z-axis rotation matrix
out=[
    -sin(th)    -cos(th)     0;
     cos(th)    -sin(th)     0;
     0           0           0;
    ];
end

