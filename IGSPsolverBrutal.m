%This is a brutal IGSP solver for an UACDPR with four cables where a 
%conostraint of position assigment is imposed 
%OUTPUT orientation

function F = IGSPsolverBrutal(MyUACDPR,position,tau_eps)

pose = [position;tau_eps(5:end)];
MyUACDPR = SetPoseAndUpdate0KIN(MyUACDPR,pose);
MyUACDPR = ComputeGravityWrench(MyUACDPR);

F = -MyUACDPR.GeomJac.Cables'*tau_eps(1:4)+MyUACDPR.Wrench;

end