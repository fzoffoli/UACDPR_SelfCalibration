function y = satur(x)

d = .5;
% n=length(x);
% y = zeros(n,1);
% for i=1:n
%    y(i,1) = 1-d./(d+abs(x(i)));
% end
% 
% end

n=length(x);
y = zeros(n,1);
for i=1:n
   if abs(x(i))>d
       y(i) = sign(x(i));
   else
       y(i) = x(i)./d;
   end
end

end