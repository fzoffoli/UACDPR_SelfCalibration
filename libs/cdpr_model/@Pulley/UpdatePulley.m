function Pulley = UpdatePulley(Pulley,GlobalAttachPoint)
%REINIT Summary of this function goes here
%   Detailed explanation goes here

Pulley=ComputePulleyFrame(Pulley,GlobalAttachPoint);

Pulley=ComputePulleyAngles(Pulley,GlobalAttachPoint);

Pulley=ComputeCableFrame(Pulley);
end

