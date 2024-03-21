function [bary,liters]=compute_bary(USV,x,disc,waterline)


points=compute_boat_limits4(USV.L,USV.W,USV.Height,[0;0;0],disc);
% zetas=zeros(1,length(points));
for k=1:length(points)
      points_temp(:,k)=x(1:3)+USV.PtoG+USV.RotMatrix*points(:,k);
% zetas(i)=USV.RotMatrix(3,:)*points(:,i)+USV.PtoG(3)+x(3);
end
points=points_temp;
direction=USV.RotMatrix*[1;0;0];
for k=1:length(-USV.L/2:disc:USV.L/2)
    points=[points,points_temp+(k)*direction*disc+USV.RotMatrix*[0;0;disc/2]*mod(k,2)];
end
q=0;
sumx=0;
sumy=0;
sumz=0;

under_idx=find(points(3,:)<=waterline);
% under_idx=find(zetas<=0.1855);

if length(under_idx)~=0

    for k=1:length(under_idx)
%         if points(3,i)<=0.1855
%             q=q+1;
%             points2(:,q)=points(:,i);
%             sumx=sumx+points(1,i);
%             sumy=sumy+points(2,i);
%             sumz=sumz+points(3,i);
%         end

        q=q+1;
            points2(:,q)=points(:,under_idx(k));
            sumx=sumx+points2(1,q);
            sumy=sumy+points2(2,q);
            sumz=sumz+points2(3,q);
    end
else
    q=0;
end


if q==0
    bary=zeros(3,1);
else
    bary=[sumx/q;sumy/q;sumz/q];
end
liters=q*10^3*disc^3;
end