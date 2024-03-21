clear all
close all

%%%%INPUT
Mass=5;
PtoG=[0;0;0];
Inertia=eye(3);
n=2;

CableEnterPosition{1}=[0;-1.5;2.5];
CableEnterPosition{2}=[0;1.5;2.5];

Radius{1}=.025;
Radius{2}=.025;

matrix1=eye(3);
FF{1}.FixedFrame{1}=matrix1(1:3,1);
FF{1}.FixedFrame{2}=matrix1(1:3,2);
FF{1}.FixedFrame{3}=matrix1(1:3,3);

matrix2=eye(3);
FF{2}.FixedFrame{1}=matrix2(1:3,1);
FF{2}.FixedFrame{2}=matrix2(1:3,2);
FF{2}.FixedFrame{3}=matrix2(1:3,3);

AttachPoints{1}=[0 -.3 .15];
AttachPoints{2}=[0 .3 .15];

DepVect=[0 1 1 0 0 0];

answF(:,1)=zeros(3,1);
answFP(:,1)=zeros(3,1);
answF_mob(:,1)=zeros(3,1);
answFP_mob(:,1)=zeros(3,1);
answM(:,1)=zeros(3,1);

gravity=9.81;


%%%Instantiation

%%%Platform
    out.m=Mass;
    out.PtoG=PtoG;
    for i=1:length(AttachPoints)
        out.points{i}=AttachPoints{i};
    end
    out.InertialMatrix=Inertia;
    
    s.EndEffector.Platform=out;

    
%%%Trasmission
    for i=1:n
        s.Trasmission.Cable(i)=i;
        s.Trasmission.Winch(i)=i;
        out2{i}.Id=int8(i);
        out2{i}.r=Radius{i};
        out2{i}.CableEnterPosition=CableEnterPosition{i};
        out2{i}.FixedFrame=FF{i}.FixedFrame;
    end


    s.Trasmission.Pulley=out2;
%     clear out
    s.DependencyVect=DepVect;
    
%%%Forces   

        Fg=[0;0;-s.EndEffector.Platform.m*gravity];
        
        s.GlobalForces.Forces=[Fg,answF];
        s.GlobalForces.Points=[s.EndEffector.Platform.PtoG,answFP];
        
        s.LocalForces.Forces=answF;
        s.LocalForces.Points=answFP;
        
        s.GlobalMoments=sum(answM,2);
    
    
