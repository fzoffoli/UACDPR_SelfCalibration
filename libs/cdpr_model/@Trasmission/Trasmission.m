classdef Trasmission
    %TRASMISSION Summary of this class goes here
    %   Detailed explanation goes here
    
%     properties  (SetAccess= ?UACDPR)
    properties  (SetAccess= public)
        Cable cell
        Pulley cell
        Winch  cell
        
        TMatrixes cell
        
        CableLengths double
        CableLengths_dt double
        CableLengths_ddt double
        
        Tangency_dt double
        Swivel_dt double
       
    end
    
%     properties (Dependent)
%         CableTs double
%     end
    
    properties (SetAccess=?UACDPR)
         CableTensions double
         LowerBound double
    end


%%% PUBLIC METHODS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    methods
        
        % Update Trasmission info
        obj = UpdateTrasmission(obj,AttachPoints)
        %INPUT: global-referenced Platform Attach Points
        %OUTPUT: Call for Cable, Pulley, Winch update, CableLengths Update
        
        obj = UpdateTMatrixes(obj)
        
        obj= ComputeCableDerivatives(obj,GeomJacl,GeomJacl_dt,Twist,Twist_dt)
        
        obj = ComputeAngleDerivatives(obj,Twist,GeomJac)

        
    end   
%%% CONSTRUCTOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    methods%(Access = ?UACDPR)
        function obj = Trasmission(In)
            for i=1:length(In.Cable)
            obj.Cable{i}=Cable(In.Cable(i));
            obj.Pulley{i}=Pulley(In.Pulley{i});
            obj.Winch{i}=Winch(In.Winch(i));
            end
            
            
            obj.CableLengths=zeros(length(obj.Cable),1);
        end
        

    end
%%% GETTERs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%     methods
%         function a=get.CableTs(obj)
%             for i=1:length(obj.Pulley)
%             a(:,i)=obj.Pulley{i}.CableFrame{1};
%             end
%         end
%     end
%%% SETTERs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% PRIVATE METHODS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   

end

