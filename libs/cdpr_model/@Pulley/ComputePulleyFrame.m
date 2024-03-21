function Pulley=ComputePulleyFrame(Pulley,GlobalAttachPoint)

    %INPUT:  Swivel axis, Cable Enter Position and End Effector pose
    %OUTPUT: Pulley-solidal Frame (u,w,k)
        Pulley.DA=GlobalAttachPoint-Pulley.CableEnterPosition;                        %Call to another function
        PulleyPlaneNorm=cross(Pulley.SwivelAxis,Pulley.DA)/norm(cross(Pulley.SwivelAxis,Pulley.DA));            %normalized cross product between swivel axis and DA
        u=cross(PulleyPlaneNorm,Pulley.SwivelAxis)/norm(cross(PulleyPlaneNorm,Pulley.SwivelAxis));        %normalized cross product between pulley-plane versor and swivel axis
        
        temp{1}=u;
        temp{2}=PulleyPlaneNorm;
        temp{3}=Pulley.SwivelAxis;
        Pulley.PulleyFrame=temp;

end