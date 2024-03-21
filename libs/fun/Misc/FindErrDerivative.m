function A=FindErrDerivative(contr2,s)

L=[zeros(3)                     eye(3);
   -contr2.k1*contr2.Gamma  -(contr2.k1+contr2.Gamma)];

d = 0.5;

n=length(s);

NL3=zeros(3);
NL4=zeros(3);
for i=1:n
   if abs(s(i))>d
        NL3(i,i)=0;
        NL4(i,i)=0;
   else
        NL3(i,i)=-contr2.k2*contr2.Gamma(i,i)./d;
        NL4(i,i)=-contr2.k2./d;
   end
end

NL=[zeros(3) zeros(3);
    NL3 NL4];

A=L+NL;