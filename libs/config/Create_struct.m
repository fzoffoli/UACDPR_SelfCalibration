function s = Create_struct(Mass,PtoG,Inertia,n,CableEnterPosition1,CableEnterPosition2,Radius1,Radius2,matrix1,matrix2,AttachPoints1,AttachPoints2,DepVect,gravity)
    

Radius{1}=Radius1;
Radius{2}=Radius2;


CableEnterPosition{1}=CableEnterPosition1;
CableEnterPosition{2}=CableEnterPosition2;

AttachPoints{1}=AttachPoints1;
AttachPoints{2}=AttachPoints2;

FF{1}.FixedFrame{1}=matrix1(1:3,1);
FF{1}.FixedFrame{2}=matrix1(1:3,2);
FF{1}.FixedFrame{3}=matrix1(1:3,3);


FF{2}.FixedFrame{1}=matrix2(1:3,1);
FF{2}.FixedFrame{2}=matrix2(1:3,2);
FF{2}.FixedFrame{3}=matrix2(1:3,3);

answF(:,1)=zeros(3,1);
answFP(:,1)=zeros(3,1);
answF_mob(:,1)=zeros(3,1);
answFP_mob(:,1)=zeros(3,1);
answM(:,1)=zeros(3,1);

out.m=Mass;
    out.PtoG=PtoG;
    for i=1:length(AttachPoints)
        out.points{i}=AttachPoints{i};
    end
    out.InertialMatrix=Inertia;
    
    s.EndEffector.Platform=out;
    clear out
%%%Trasmission

    for i=1:n
        s.Trasmission.Cable(i)=i;
        s.Trasmission.Winch(i)=i;
        out{i}.Id=int8(i);
        out{i}.r=Radius{i};
        out{i}.CableEnterPosition=CableEnterPosition{i};
        out{i}.FixedFrame=FF{i}.FixedFrame;
    end


    s.Trasmission.Pulley=out;
    clear out
    s.DependencyVect=DepVect;
    
%%%Forces   

        Fg=[0;0;-s.EndEffector.Platform.m*gravity];
        
        s.GlobalForces.Forces=[Fg,answF];
        s.GlobalForces.Points=[s.EndEffector.Platform.PtoG,answFP];
        
        s.LocalForces.Forces=answF;
        s.LocalForces.Points=answFP;
        
        s.GlobalMoments=sum(answM,2);
    
end

