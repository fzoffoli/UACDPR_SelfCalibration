function buildstructure(btn,tg)

s.DependencyVect=[tg.Children(1).Children(24).Value;
    tg.Children(1).Children(23).Value;
    tg.Children(1).Children(22).Value;
    tg.Children(1).Children(21).Value;
    tg.Children(1).Children(20).Value;
    tg.Children(1).Children(19).Value; ].';

s.EndEffector.Platform.m=tg.Children(1).Children(41).Value;

s.EndEffector.Platform.PtoG=[tg.Children(1).Children(34).Value;
                            tg.Children(1).Children(33).Value;
                            tg.Children(1).Children(32).Value;];
                        
Ixx=tg.Children(1).Children(14).Value;
Ixy=tg.Children(1).Children(13).Value;
Ixz=tg.Children(1).Children(12).Value;
Iyy=tg.Children(1).Children(9).Value;
Iyz=tg.Children(1).Children(8).Value;
Izz=tg.Children(1).Children(6).Value;

s.EndEffector.Platform.InertialMatrix=[Ixx Ixy Ixz;
                                        Ixy Iyy Iyz;
                                        Ixz Iyz Izz];
                                    
robotname= tg.Children(1).Children(45).Value;                                   
cablesnumber=tg.Children(1).Children(43).Value;                               

for i=1:cablesnumber

    s.Trasmission.Cable=1:cablesnumber;
    s.Trasmission.Winch=1:cablesnumber;
    s.Trasmission.Pulley{i}.Id=i;
    
    s.Trasmission.Pulley{i}.r=tg.Children(2).Children(cablesnumber-(i-1)).Children(25).Value;
    
    s.Trasmission.Pulley{i}.CableEnterPosition=[tg.Children(2).Children(cablesnumber-(i-1)).Children(21).Value;
                                                tg.Children(2).Children(cablesnumber-(i-1)).Children(20).Value;
                                                tg.Children(2).Children(cablesnumber-(i-1)).Children(19).Value];
                                            
    s.Trasmission.Pulley{i}.FixedFrame{1}=[tg.Children(2).Children(cablesnumber-(i-1)).Children(15).Value;
                                            tg.Children(2).Children(cablesnumber-(i-1)).Children(14).Value;
                                            tg.Children(2).Children(cablesnumber-(i-1)).Children(13).Value];
                                        
    s.Trasmission.Pulley{i}.FixedFrame{2}=[tg.Children(2).Children(cablesnumber-(i-1)).Children(9).Value;
                                            tg.Children(2).Children(cablesnumber-(i-1)).Children(8).Value;
                                            tg.Children(2).Children(cablesnumber-(i-1)).Children(7).Value];
                
    s.Trasmission.Pulley{i}.FixedFrame{3}=[tg.Children(2).Children(cablesnumber-(i-1)).Children(3).Value;
                                            tg.Children(2).Children(cablesnumber-(i-1)).Children(2).Value;
                                            tg.Children(2).Children(cablesnumber-(i-1)).Children(1).Value];
                                        
    s.EndEffector.Platform.points{i}=[tg.Children(3).Children(cablesnumber-(i-1)).Children(3).Value, tg.Children(3).Children(cablesnumber-(i-1)).Children(2).Value, tg.Children(3).Children(cablesnumber-(i-1)).Children(1).Value];
    
    
end
    answF=[0;0;0];
    answF2=[0;0;0];
    answM=[0;0;0];
    answFP=[0;0;0];
    answFP2=[0;0;0];
    
    acc_gravity=tg.Children(1).Children(39).Value;
    Fg=[0;0;-s.EndEffector.Platform.m*acc_gravity];

    s.GlobalForces.Forces=[Fg,answF];
    s.GlobalForces.Points=[s.EndEffector.Platform.PtoG,answFP];

    s.LocalForces.Forces=answF2;
    s.LocalForces.Points=answFP2;

    s.GlobalMoments=sum(answM,2);
p=s;
save(robotname,'s')
end