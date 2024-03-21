function Pose= Find_pose_track(UACDPR,CableLengths,Guess,opts,t,dt)
global temp flag_save
% Guess=[-2.186458521478146;-12.375783670095256;7.514376006926935;6.912073388301342e+02];
tic
Pose_f = fsolve(@(Pose_f) fun(Pose_f,UACDPR,CableLengths,[0;0;0]),Guess,opts.FsolveGrad8);
toc
tic
[Pose_f2,~]=NewtonRhapson(@(Pose_f) comp_fun(Pose_f,UACDPR,CableLengths,[0;0;0]),@(Pose_f) comp_jac(Pose_f,UACDPR,[0;0;0]),Guess,10^(-5),10^(-3),10^(3));
toc
if max((Pose_f-Pose_f2)/Pose_f*100)>5
    qqqq=1;
end

Pose=[Pose_f;[0;0;0]];
if flag_save==0
 temp(4:9,int64(t/dt)+1)=[Pose_f;Pose_f2];
end
end
%%% FUNCTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [F,Jac]=fun(Pose_f,UACDPR,CableLengths,Pose_d)
    %Geometry Computation and Wrench Generation
    
    UACDPR    = SetPoseAndUpdate0KIN(UACDPR,[Pose_f;Pose_d]);
   
%%%%% Fsolve system %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
  F=UACDPR.Trasmission.CableLengths(1:4)-CableLengths;
   
%%%% Computation of Jacobians %%%%%%%%%
   

  Jac=[UACDPR.AnalJac.Cables(1:4,1:3)];
end

function [F]=comp_fun(Pose_f,UACDPR,CableLengths,Pose_d)
UACDPR    = SetPoseAndUpdate0KIN(UACDPR,[Pose_f;Pose_d]);
F=UACDPR.Trasmission.CableLengths(1:4)-CableLengths;
 
end

function [Jac]=comp_jac(Pose_f,UACDPR,Pose_d)
UACDPR    = SetPoseAndUpdate0KIN(UACDPR,[Pose_f;Pose_d]);
Jac=[UACDPR.AnalJac.Cables(1:4,1:3)];
end
