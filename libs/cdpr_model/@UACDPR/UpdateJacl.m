function UACDPR = UpdateJacl(UACDPR,LocalAttachPoints,Pulley,D)
        %INPUT: End-Effector attach point, Cable Local Frames first versor, D Matrix
        %OUTPUT: Kinematic and analitic cable Jacobian

    for i=1:length(LocalAttachPoints)
        
       
        UACDPR.GeomJac.Cables(i,1:6)=        [Pulley{i}.CableFrame{1};
        cross((LocalAttachPoints{i}),Pulley{i}.CableFrame{1})].';

    end
    
    UACDPR.AnalJac.Cables=UACDPR.GeomJac.Cables*D;

end

