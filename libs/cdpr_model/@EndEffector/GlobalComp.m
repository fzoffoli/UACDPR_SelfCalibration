function obj = GlobalComp(obj)
        %INPUT: Orientation and coordinates
        %OUTPUT: Update global attach points coordinates
    if nargin==1
        for i=1:length(obj.Platform_Param.AttachPoint)
           obj.GlobalAttachPoints{i}=obj.RotMatrix*obj.Platform_Param.AttachPoint{i}+obj.Pose(1:3);
        end
    end

    
end