function line = drawline_animated(p1,p2,line)
if length(p1)==2

        addpoints(line,p1(1),p1(2)); 
        addpoints(line,p2(1),p2(2)); 
elseif length(p1)==3
        addpoints(line,p1(1),p1(2),p1(3)); 
        addpoints(line,p2(1),p2(2),p2(3)); 
else
    error('wrong number of inputs')
end
end

