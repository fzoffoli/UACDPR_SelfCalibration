function [UACDPR] = Init(UACDPR,s)

UACDPR.CablesNumber = nnz(UACDPR.DependencyVect);
UACDPR.FreeNumber = 6-UACDPR.CablesNumber;

UACDPR = UACDPR.GeneratePermutMatrix;

[~,temp]=size(UACDPR.LocalForces);
for i=1:temp
   localmoments=skew(UACDPR.LocalForces.Points(:,i))*UACDPR.LocalForces.Forces(:,i); 
end

UACDPR.GlobalMoments=s.GlobalMoments+localmoments;

end

