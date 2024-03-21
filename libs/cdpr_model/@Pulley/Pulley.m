classdef Pulley
    %Pulley object
    
    properties  (SetAccess= private)
        Radius (1,1) double
%         CenterPosition double
        CableEnterPosition (3,1) double       %3x1 Point of first contact with cable (cable coming from winch)
        CableEnterPosition_Param (3,1) double       %3x1 Point of first contact with cable (cable coming from winch), w.r.t. robot frame
        CableExitPosition (3,1) double        %3x1 Point of last contact with cable (cable going to platform)

        SwivelAxis (3,1) double               %3x1 Swivel-Axis Versor
        SwivelAngle (1,1) double              %Swivel angle
        TangencyAngle (1,1) double            %Tangency angle

        SwivelAngle_dpose   (1,6) double      %Swivel angle pose derivative
        TangencyAngle_dpose (1,6) double      %Tangency angle pose derivative

        PulleyFrame (1,3) cell                %1x3 Cell array containing base versor for pulley-solidal frame
        FixedFrame (1,3) cell                 %1x3 Cell array cointaining base versor for fixed frame attached to CableEnterPosition
        FixedFrame_Param (1,3) cell           %1x3 Cell array cointaining base versor for fixed frame attached to CableEnterPosition, w.r.t. robot frame
        CableFrame (1,3) cell                 %1x3 Cell array containing base versor for cable-solidal frame attached to CableExitPosition

        PulleyFrame_dpose (1,3) cell                %1x3 Cell array containing the pose derivative of base versor for pulley-solidal frame
        CableFrame_dpose (1,3) cell                 %1x3 Cell array containing the pose derivative of base versor for cable-solidal frame attached to CableExitPosition
        
        DA (3,1) double                       %3x1 Vector representing link between CableEnterPosition and Platform.AttachPoint
        Id (1,1) int8                         %Identification number
    end


%%% PUBLIC METHODS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     methods  

%               Update Pulley info
                obj=UpdatePulley(obj,AttachPoints)
%               INPUT: Fixed Frame, Cable Enter Position, Radius and
%               EndEffector pose
%               OUTPUT:Pulley Frame, Pulley angles, Cable Frame

                obj=ChangeCableEnterPoint(obj,NewPoint)
                
                obj=ChangeFixedFrame(obj,NewFrame)

                obj=ComputePulley_dpose(obj,UACDPR)
     end   
    
%%% CONSTRUCTOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

methods%(Access = ?Trasmission)
        function obj = Pulley(In)
            
                obj.Radius=In.r;
                obj.PulleyFrame={[0;0;0],[0;0;0],[0;0;0]};
                obj.CableFrame={[0;0;0],[0;0;0],[0;0;0]};
                obj.CableEnterPosition= In.CableEnterPosition;
                obj.FixedFrame=In.FixedFrame;
                obj.CableEnterPosition_Param= In.CableEnterPosition;
                obj.FixedFrame_Param=In.FixedFrame;
                obj.SwivelAxis=obj.FixedFrame{3};
                obj.Id=In.Id;
        end        
    end

%%% GETTERs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    
%%% PRIVATE METHODS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     methods (Access=private)
                      
%               Local Frame Computation 
                obj=ComputePulleyFrame(obj,EndEffector)
%               INPUT:  Swivel axis, Cable Enter Position and End Effector pose
%               OUTPUT: Pulley-solidal Frame

               
%               Pulley Angle Computation
                obj=ComputePulleyAngles(obj,GlobalAttachPoint)
%               INPUT: PulleyFrame and AttachPoint
%               OUTPUT: Pulley Angles


%               Cable Frame Computation
                obj=ComputeCableFrame(obj)
%               INPUT: PulleyFrame and Tangency Angle
%               OUTPUT: Cable Frame and Cable Exit Position



    end  

end

