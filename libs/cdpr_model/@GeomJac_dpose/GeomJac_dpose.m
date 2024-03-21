classdef GeomJac_dpose
    %Geometric Jacobians derivatives wrt pose
    
    properties (SetAccess= ?UACDPR)
        Cables
        Swivel double
        Tangency double
    end

    methods

        function obj=GeomJac_dpose(DepVect)
            obj.Cables=zeros(nnz(DepVect),6,6);
            obj.Swivel=zeros(nnz(DepVect),6,6);
            obj.Tangency=zeros(nnz(DepVect),6,6);
        end

    end
end