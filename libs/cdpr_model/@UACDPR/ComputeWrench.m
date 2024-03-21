% This method compute the sum of wrenches from parameters (forces that are
% defined in config time) and external disturbances.
% 
% The result can be found in UACDPR.Wrench.

function UACDPR = ComputeWrench(UACDPR,Disturb)

Moments=UACDPR.GlobalMoments;

[~,temp1]=size(UACDPR.GlobalForces.Points);

% Computation of moments deriving from global forces
for i=1:temp1
    Moments = Moments + skew(UACDPR.RotMatrix_*UACDPR.GlobalForces.Points(:,i))*UACDPR.GlobalForces.Forces(:,i);
end

LocalForces=UACDPR.RotMatrix_*sum(UACDPR.LocalForces.Forces,2);

Forces=sum(UACDPR.GlobalForces.Forces,2)+LocalForces;

UACDPR.Wrench=[Forces;Moments]+Disturb;

end

