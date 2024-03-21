%Cable Frame computation
function Pulley = ComputeCableFrame(Pulley)
%            INPUT: PulleyFrame and Tangency Angle
%            OUTPUT: Cable Frame and Cable Exit Position (t,w,n)
    n=cos(Pulley.TangencyAngle)*Pulley.PulleyFrame{1}+sin(Pulley.TangencyAngle)*Pulley.PulleyFrame{3};
    t=sin(Pulley.TangencyAngle)*Pulley.PulleyFrame{1}-cos(Pulley.TangencyAngle)*Pulley.PulleyFrame{3};
    temp{1}=t;
    temp{2}=Pulley.PulleyFrame{2};
    temp{3}=n;
    Pulley.CableFrame=temp;

%             Pulley.CableFrame{1}=t;
%             Pulley.CableFrame{2}=Pulley.PulleyFrame{2};
%             Pulley.CableFrame{3}=n; %%n è diretto verso l'interno della puleggia
    Pulley.CableExitPosition=Pulley.CableEnterPosition+Pulley.Radius*(Pulley.PulleyFrame{1}+Pulley.CableFrame{3});
end

