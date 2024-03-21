function [out] = dRx(th)
%Derivative wrt th of elementary x-axis rotation matrix
out=[
    0      0             0;
    0     -sin(th)     -cos(th);
    0      cos(th)     -sin(th);
    ];
end

