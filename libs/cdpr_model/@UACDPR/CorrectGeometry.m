function UACDPR = CorrectGeometry(UACDPR,Pose_frame)

Frame_angle = Pose_frame(4:6);
Robot_Position = Pose_frame(1:3);
rot = RotMatrixTaitBryan(Frame_angle);

for i=1:length(UACDPR.Trasmission.Pulley)

%     NewPoint = Robot_Position + rot*UACDPR.Trasmission.Pulley{i}.CableEnterPosition_Relative;
    NewPoint = Robot_Position + rot*(UACDPR.Trasmission.Pulley{i}.CableEnterPosition_Param);
    UACDPR.Trasmission.Pulley{i} = ChangeCableEnterPoint(UACDPR.Trasmission.Pulley{i},NewPoint);

    for j=1:3
        NewFrame{j} =rot*UACDPR.Trasmission.Pulley{i}.FixedFrame_Param{j};
    end
    UACDPR.Trasmission.Pulley{i} = ChangeFixedFrame(UACDPR.Trasmission.Pulley{i},NewFrame);
end


end

