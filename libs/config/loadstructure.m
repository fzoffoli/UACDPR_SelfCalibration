function loadstructure(btn,tg)

load(tg.Children(1).Children(47).Value);
% robotname= tg.Children(1).Children(44).Value; 

tg.Children(1).Children(24).Value=s.DependencyVect(1);
tg.Children(1).Children(23).Value=s.DependencyVect(2);
tg.Children(1).Children(22).Value=s.DependencyVect(3);
tg.Children(1).Children(21).Value=s.DependencyVect(4);
tg.Children(1).Children(20).Value=s.DependencyVect(5);
tg.Children(1).Children(19).Value=s.DependencyVect(6);
% 
% 
% s.DependencyVect=[tg.Children(1).Children(23).Value;
%     tg.Children(1).Children(22).Value;
%     tg.Children(1).Children(21).Value;
%     tg.Children(1).Children(20).Value;
%     tg.Children(1).Children(19).Value;
%     tg.Children(1).Children(18).Value; ].';
tg.Children(1).Children(41).Value=s.EndEffector.Platform.m;

% s.EndEffector.Platform.m=tg.Children(1).Children(40).Value;
tg.Children(1).Children(34).Value=s.EndEffector.Platform.PtoG(1);
tg.Children(1).Children(33).Value=s.EndEffector.Platform.PtoG(2);
tg.Children(1).Children(32).Value=s.EndEffector.Platform.PtoG(3);
% 
% s.EndEffector.Platform.PtoG=[tg.Children(1).Children(33).Value;
%                             tg.Children(1).Children(32).Value;
%                             tg.Children(1).Children(31).Value;];

tg.Children(1).Children(14).Value=s.EndEffector.Platform.InertialMatrix(1,1);
tg.Children(1).Children(13).Value=s.EndEffector.Platform.InertialMatrix(1,2);
tg.Children(1).Children(12).Value=s.EndEffector.Platform.InertialMatrix(1,3);
tg.Children(1).Children(9).Value=s.EndEffector.Platform.InertialMatrix(2,2);
tg.Children(1).Children(8).Value=s.EndEffector.Platform.InertialMatrix(2,3);
tg.Children(1).Children(6).Value=s.EndEffector.Platform.InertialMatrix(3,3);
% 
% Ixx=tg.Children(1).Children(13).Value;
% Ixy=tg.Children(1).Children(12).Value;
% Ixz=tg.Children(1).Children(11).Value;
% Iyy=tg.Children(1).Children(8).Value;
% Iyz=tg.Children(1).Children(7).Value;
% Izz=tg.Children(1).Children(5).Value;
% 
% s.EndEffector.Platform.InertialMatrix=[Ixx Ixy Ixz;
%                                         Ixy Iyy Iyz;
%                                         Ixz Iyz Izz];

tg.Children(1).Children(43).Value=length(s.Trasmission.Cable);
                                  
cablesnumber=tg.Children(1).Children(43).Value;
createpulleys(tg.Children(1).Children(4),cablesnumber,tg,tg.Children(2))
createplatform_ui(tg.Children(1).Children(5),cablesnumber,tg,tg.Children(3))

for i=1:cablesnumber

%     s.Trasmission.Cable=1:cablesnumber;
%     s.Trasmission.Winch=1:cablesnumber;
%     s.Trasmission.Pulley{i}.Id=i;
    
    tg.Children(2).Children(cablesnumber-(i-1)).Children(25).Value=s.Trasmission.Pulley{i}.r;

%     s.Trasmission.Pulley{i}.r=tg.Children(2).Children(cablesnumber-(i-1)).Children(25).Value;
    
    tg.Children(2).Children(cablesnumber-(i-1)).Children(21).Value=s.Trasmission.Pulley{i}.CableEnterPosition(1);
    tg.Children(2).Children(cablesnumber-(i-1)).Children(20).Value=s.Trasmission.Pulley{i}.CableEnterPosition(2);
    tg.Children(2).Children(cablesnumber-(i-1)).Children(19).Value=s.Trasmission.Pulley{i}.CableEnterPosition(3);

%     s.Trasmission.Pulley{i}.CableEnterPosition=[tg.Children(2).Children(cablesnumber-(i-1)).Children(21).Value;
%                                                 tg.Children(2).Children(cablesnumber-(i-1)).Children(20).Value;
%                                                 tg.Children(2).Children(cablesnumber-(i-1)).Children(19).Value];

    temp=s.Trasmission.Pulley{i}.FixedFrame{1};
    tg.Children(2).Children(cablesnumber-(i-1)).Children(15).Value=temp(1);
    tg.Children(2).Children(cablesnumber-(i-1)).Children(14).Value=temp(2);
    tg.Children(2).Children(cablesnumber-(i-1)).Children(13).Value=temp(3);
    clear temp
%                                             
%     s.Trasmission.Pulley{i}.FixedFrame{1}=[tg.Children(2).Children(cablesnumber-(i-1)).Children(15).Value;
%                                             tg.Children(2).Children(cablesnumber-(i-1)).Children(14).Value;
%                                             tg.Children(2).Children(cablesnumber-(i-1)).Children(13).Value];
    
    temp=s.Trasmission.Pulley{i}.FixedFrame{2};
    tg.Children(2).Children(cablesnumber-(i-1)).Children(9).Value=temp(1);
    tg.Children(2).Children(cablesnumber-(i-1)).Children(8).Value=temp(2);
    tg.Children(2).Children(cablesnumber-(i-1)).Children(7).Value=temp(3);
    clear temp
    
%     s.Trasmission.Pulley{i}.FixedFrame{2}=[tg.Children(2).Children(cablesnumber-(i-1)).Children(9).Value;
%                                             tg.Children(2).Children(cablesnumber-(i-1)).Children(8).Value;
%                                             tg.Children(2).Children(cablesnumber-(i-1)).Children(7).Value];

    temp=s.Trasmission.Pulley{i}.FixedFrame{3};
    tg.Children(2).Children(cablesnumber-(i-1)).Children(3).Value=temp(1);
    tg.Children(2).Children(cablesnumber-(i-1)).Children(2).Value=temp(2);
    tg.Children(2).Children(cablesnumber-(i-1)).Children(1).Value=temp(3);
    clear temp
% 
%     s.Trasmission.Pulley{i}.FixedFrame{3}=[tg.Children(2).Children(cablesnumber-(i-1)).Children(3).Value;
%                                             tg.Children(2).Children(cablesnumber-(i-1)).Children(2).Value;
%                                             tg.Children(2).Children(cablesnumber-(i-1)).Children(1).Value];
    temp=s.EndEffector.Platform.points{i};
    tg.Children(3).Children(cablesnumber-(i-1)).Children(3).Value=temp(1);
    tg.Children(3).Children(cablesnumber-(i-1)).Children(2).Value=temp(2);
    tg.Children(3).Children(cablesnumber-(i-1)).Children(1).Value=temp(3);
    clear temp
%     
%     s.EndEffector.Platform.points{i}=[tg.Children(3).Children(cablesnumber-(i-1)).Children(3).Value;
%                                             tg.Children(3).Children(cablesnumber-(i-1)).Children(2).Value;
%                                             tg.Children(3).Children(cablesnumber-(i-1)).Children(1).Value];
    
    
end


    tg.Children(1).Children(39).Value=abs(s.GlobalForces.Forces(3,1)/s.EndEffector.Platform.m);
    

    answF=[0;0;0];
    answF2=[0;0;0];
    answM=[0;0;0];
    answFP=[0;0;0];
    answFP2=[0;0;0];
    
%     acc_gravity=tg.Children(1).Children(37).Value;
%     Fg=[0;0;-s.EndEffector.Platform.m*acc_gravity];
% 
%     s.GlobalForces.Forces=[Fg,answF];
%     s.GlobalForces.Points=[s.EndEffector.Platform.PtoG,answFP];
% 
%     s.LocalForces.Forces=answF2;
%     s.LocalForces.Points=answFP2;
% 
%     s.GlobalMoments=sum(answM,2);
% p=s;
% save test_UUUUIIIII p
end