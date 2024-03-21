function UACDPR = ComputeAngleJac_dt(UACDPR,LocalAttachPoints,Cable,Pulley,D)

    Tangency=UACDPR.GeomJac.Tangency;
    Swivel=UACDPR.GeomJac.Swivel;
    
    for i=1:length(LocalAttachPoints)
        Pulley=UACDPR.Trasmission.Pulley{i};
       
        UACDPR.GeomJac.u(1:6,i)=        [Pulley.PulleyFrame{1};
        cross((LocalAttachPoints{i}),Pulley.PulleyFrame{1})];
    
    
        A_w=[zeros(3,6);zeros(3,3) skew(LocalAttachPoints{i})*skew(Pulley.PulleyFrame{2})];
        UACDPR.GeomJac.Swivel_dv=1/dot(Pulley.DA,Pulley.PulleyFrame{1})*(-2*UACDPR.GeomJac.u(1:6,i)*Swivel(i,1:6)+A_w);
        
    end

    for i=1:length(LocalAttachPoints)
        Pulley=UACDPR.Trasmission.Pulley{i};
           
        A_n=[zeros(3,6);zeros(3,3) skew(LocalAttachPoints{i})*skew(Pulley.CableFrame{3})];
        
        UACDPR.GeomJac.Tangency_dv=1/norm(Cable{i}.CableVector)*...
        (dot(Pulley.DA,Pulley.PulleyFrame{1})*cos(Pulley.TangencyAngle)*Swivel(i,1:6).'*Swivel(i,1:6)...
        -Pulley.Radius*Tangency(i,1:6).'*Tangency(i,1:6)...
        -2*UACDPR.GeomJac.Cables(i,1:6).'*Tangency(i,1:6)+A_n);
        
    end
   
    

end

