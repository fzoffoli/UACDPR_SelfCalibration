function [out] = Rx(th)
%Elementary x-axis rotation matrix
out=[
    1      0             0;
    0     cos(th)     -sin(th);
    0     sin(th)     cos(th);
    ];
end

