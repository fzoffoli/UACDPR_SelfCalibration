function UACDPR = UpdateJacl_dt(UACDPR,Twist,LocalAttachPoints,GeomJac,Pulley,Cable,D,D_dt)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here



    for i=1:size(GeomJac.Swivel,1)
        
            A = sin(Pulley{i}.TangencyAngle)*dot(Pulley{i}.DA,Pulley{i}.PulleyFrame{1})*GeomJac.Swivel(i,:).'*GeomJac.Swivel(i,:);
            B = norm(Cable{i}.CableVector)*GeomJac.Tangency(i,:).'*GeomJac.Tangency(i,:);
            C = [zeros(3,6); zeros(3) skew(LocalAttachPoints{i})*skew(Pulley{i}.CableFrame{1})];

            UACDPR.GeomJac.Cables_dt(i,1:6) = ((A+B+C)*Twist).';
            UACDPR.GeomJac.zita{i} = (A+B+C);
    
        
    end
    
%     UACDPR.AnalJac.Cables_dt=D_dt.'*GeomJac.Cables+D.'*UACDPR.GeomJac.Cables_dt;
UACDPR.AnalJac.Cables_dt=GeomJac.Cables*D_dt+UACDPR.GeomJac.Cables_dt*D;
end

