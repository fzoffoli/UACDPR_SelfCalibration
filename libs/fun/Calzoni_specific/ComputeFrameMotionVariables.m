function [eps] = ComputeFrameMotionVariables(obj,Pose_frame)

Frame_angle = Pose_frame(4:6);

O_c=Pose_frame(1:3);
LocalPoints=obj.EndEffector.LocalAttachPoints_;

for i=1:length(obj.Trasmission.Pulley)
    
a_i=obj.EndEffector.GlobalAttachPoints{i}+O_c;
t_i=obj.Trasmission.Pulley{i}.CableFrame{1};


eps.cable_s(:,i)=[t_i; skew(a_i-O_c)*t_i];
% eps.cable_s(:,i)=[t_i; skew(a_i)*t_i];

eps.swiv_s(:,i)=[obj.Trasmission.Pulley{i}.CableFrame{2};
        cross(a_i-O_c,obj.Trasmission.Pulley{i}.CableFrame{2})]./dot(obj.Trasmission.Pulley{i}.DA,obj.Trasmission.Pulley{i}.PulleyFrame{1});
    
eps.tang_s(:,i)=[obj.Trasmission.Pulley{i}.CableFrame{3};
        cross(a_i-O_c,obj.Trasmission.Pulley{i}.CableFrame{3})]./norm(obj.Trasmission.Cable{i}.CableVector);
% end
% 
% for i=1:length(obj.Trasmission.Pulley)
   
   A_t=[zeros(3,6);
       zeros(3,3)  skew(LocalPoints{i})*skew(t_i)]; 
    
   eps.cable_apex{i} = norm(obj.Trasmission.Cable{i}.CableVector)*obj.GeomJac.Tangency(i,:).'*obj.GeomJac.Tangency(i,:)+...
                            sin(obj.Trasmission.Pulley{i}.TangencyAngle)*dot(obj.Trasmission.Pulley{i}.DA,obj.Trasmission.Pulley{i}.PulleyFrame{1})*obj.GeomJac.Swivel(i,:).'*obj.GeomJac.Swivel(i,:)+...
                            A_t;
                        
   A_t_s=[zeros(3,3) skew(t_i);
       (skew(t_i)).'  (skew(a_i)-skew(O_c))*skew(t_i)];
   
   eps.cable_apex_s{i}=norm(obj.Trasmission.Cable{i}.CableVector)*eps.tang_s(:,i)*eps.tang_s(:,i).'+...
                            sin(obj.Trasmission.Pulley{i}.TangencyAngle)*dot(obj.Trasmission.Pulley{i}.DA,obj.Trasmission.Pulley{i}.PulleyFrame{1})*eps.swiv_s(:,i)*eps.swiv_s(:,i).'+...
                            A_t_s;
                            
   A_t_x=[zeros(3,3) skew(t_i);
          zeros(3,3)  skew(LocalPoints{i})*skew(t_i)];
      
   eps.cable_x{i}=norm(obj.Trasmission.Cable{i}.CableVector)*obj.GeomJac.Tangency(i,:).'*eps.tang_s(:,i).'+...
                sin(obj.Trasmission.Pulley{i}.TangencyAngle)*dot(obj.Trasmission.Pulley{i}.DA,obj.Trasmission.Pulley{i}.PulleyFrame{1})*obj.GeomJac.Swivel(i,:).'*eps.swiv_s(:,i).'+...
                A_t_x;
            
end


end
