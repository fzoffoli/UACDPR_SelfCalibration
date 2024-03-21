        % Pulley Angle computation
function Pulley=ComputePulleyAngles(Pulley,AttachPoints)
    %INPUT: PulleyFrame and End Effector pose
    %OUTPUT: Pulley Angles
    Pulley.DA=AttachPoints-Pulley.CableEnterPosition;
%     Pulley.SwivelAngle=atan2(dot(Pulley.FixedFrame{1},Pulley.DA),dot(Pulley.FixedFrame{2},Pulley.DA));
    Pulley.SwivelAngle=atan2(dot(Pulley.FixedFrame{2},Pulley.DA),dot(Pulley.FixedFrame{1},Pulley.DA));

    temp1=dot(Pulley.DA,Pulley.PulleyFrame{3});
    temp2=dot(Pulley.DA,Pulley.PulleyFrame{1});
    temp3=2*Pulley.Radius;
    
%     ti = temp1/temp2+sqrt((temp1/temp2)^2+1-temp3/temp2);
%     Pulley.TangencyAngle = 2*atan2(2*ti,(1-ti^2));
    Pulley.TangencyAngle=2*atan(temp1/temp2+sqrt((temp1/temp2)^2+1-temp3/temp2));
%     if (~isreal(Pulley.TangencyAngle))||(~isreal(Pulley.SwivelAngle))
%         a=1;
%     end
end
