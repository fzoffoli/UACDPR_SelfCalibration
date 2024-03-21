function out=compute_poly_fromcoeff(coeff,x)
for i=1:length(x)
   out(i)=0;
   for j=1:length(coeff)
       out(i)=out(i)+coeff(j)*x(i)^(j-1);
   end
end