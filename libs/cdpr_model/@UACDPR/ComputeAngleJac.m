function UACDPR = ComputeAngleJac(UACDPR,LocalAttachPoints,Cable,Pulley,D)


    UACDPR.GeomJac.Tangency=zeros(length(LocalAttachPoints),6);
    UACDPR.GeomJac.Swivel=zeros(length(LocalAttachPoints),6);
    
    for i=1:length(LocalAttachPoints)
        
       
        UACDPR.GeomJac.Tangency(i,1:6)=        [Pulley{i}.CableFrame{3};
        cross((LocalAttachPoints{i}),Pulley{i}.CableFrame{3})].'./norm(Cable{i}.CableVector);
    
        
    end
    
    
    for i=1:length(LocalAttachPoints)
        
       
        UACDPR.GeomJac.Swivel(i,1:6)=        [Pulley{i}.CableFrame{2};
        cross((LocalAttachPoints{i}),Pulley{i}.CableFrame{2})].'./dot(Pulley{i}.DA,Pulley{i}.PulleyFrame{1});
    
        
    end
    
    
    UACDPR.AnalJac.Tangency=UACDPR.GeomJac.Tangency*D;
    UACDPR.AnalJac.Swivel=UACDPR.GeomJac.Swivel*D;

end

