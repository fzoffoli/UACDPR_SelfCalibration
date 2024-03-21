%Cable Frame computation
function Pulley = ComputePulley_dpose(Pulley,UACDPR)
%            INPUT: Pulley object
%            OUTPUT: Pulley and Cable Frames (u,w,k) and (t,w,n) derivatives wrt pose
%                    Swivel and Tangency angles derivatives wrt pose

    du_dpose = Pulley.PulleyFrame{2}*UACDPR.GeomJac.Swivel(Pulley.Id,1:6)*UACDPR.EndEffector.D;
    dw_dpose = - Pulley.PulleyFrame{1}*UACDPR.GeomJac.Swivel(Pulley.Id,1:6)*UACDPR.EndEffector.D;
    dk_dpose = zeros(3,6);
    temp{1}=du_dpose;
    temp{2}=dw_dpose;
    temp{3}=dk_dpose;
    Pulley.PulleyFrame_dpose=temp;

    dt_dpose = (sin(Pulley.TangencyAngle)*Pulley.PulleyFrame{2}*UACDPR.GeomJac.Swivel(Pulley.Id,:) + ...
        Pulley.CableFrame{3}*UACDPR.GeomJac.Tangency(Pulley.Id,:))*UACDPR.EndEffector.D;  
    dw_dpose = - Pulley.PulleyFrame{1}*UACDPR.GeomJac.Swivel(Pulley.Id,1:6)*UACDPR.EndEffector.D;
    dn_dpose = (cos(Pulley.TangencyAngle)*Pulley.PulleyFrame{2}*UACDPR.GeomJac.Swivel(Pulley.Id,:) - ...
        Pulley.CableFrame{1}*UACDPR.GeomJac.Tangency(Pulley.Id,:))*UACDPR.EndEffector.D;  
    temp{1}=dt_dpose;
    temp{2}=dw_dpose;
    temp{3}=dn_dpose;
    Pulley.CableFrame_dpose=temp;

    Pulley.SwivelAngle_dpose= UACDPR.GeomJac.Swivel(Pulley.Id,1:6)*UACDPR.EndEffector.D;
    Pulley.TangencyAngle_dpose= UACDPR.GeomJac.Tangency(Pulley.Id,1:6)*UACDPR.EndEffector.D;
end

