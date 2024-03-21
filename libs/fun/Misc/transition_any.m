function [out]=transition_any(in1,in2,valuebegin,valueend,value)


a=(value-(valuebegin))/(valueend-valuebegin);
a=max(0,min(a,1));


if valuebegin<valueend
    if value<valueend
        out=in2*(a)+in1*(1-a);
    else
        out=in2;
    end
else
    if value>valueend
        out=in2*(a)+in1*(1-a);
    else
        out=in2;
    end
end
end