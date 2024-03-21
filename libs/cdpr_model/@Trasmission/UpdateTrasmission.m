function [Trasmission] = UpdateTrasmission(Trasmission,GlobalAttachPoints)

    for i=1:min([length(Trasmission.Cable),length(Trasmission.Pulley),length(Trasmission.Winch)])
      %Call for Pulley Update
      Trasmission.Pulley{i}=Trasmission.Pulley{i}.UpdatePulley(GlobalAttachPoints{i});

      %Call for Cable Update 
      Trasmission.Cable{i}=Trasmission.Cable{i}.UpdateCable(GlobalAttachPoints{i},Trasmission.Pulley{i}.CableExitPosition,Trasmission.Pulley{i}.Radius,Trasmission.Pulley{i}.TangencyAngle);
%     Trasmission.Winch(i).UpdateWinch;

      %Cable Lengths assignment
      Trasmission.CableLengths(i,1)=Trasmission.Cable{i}.Length;
        
    end
    
end

