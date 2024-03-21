function s = CreateKinVar(s)
%ADDFIRSTKINVAR Summary of this function goes here
%   Detailed explanation goes here
s.FirstKinVar=FirstKinVar(s.DependencyVect);
s.SecondKinVar=SecondKinVar(s.DependencyVect);
end

