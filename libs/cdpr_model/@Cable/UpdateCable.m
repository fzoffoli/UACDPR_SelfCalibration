        %Update Cable
function Cable=UpdateCable(Cable,AttachPoint,CableExitPosition,PulleyRadius,TangencyAngle)
        %INPUT Cable Exit Position, Global Platform Attach point, Tangency
        %angle and Pulley Radius
        %OUTPUT Update Cable Vector info and Cable Length 

    Cable.CableVector=AttachPoint-CableExitPosition;
    
    Cable.Length=PulleyRadius*(pi-TangencyAngle)+norm(Cable.CableVector);
end