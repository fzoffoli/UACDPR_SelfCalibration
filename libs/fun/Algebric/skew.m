function [out] = skew(A)
%Skew symmetric representation of a vector
out=[   0      -A(3)   A(2);        A(3)    0     -A(1);       -A(2)   A(1)    0;];
end

