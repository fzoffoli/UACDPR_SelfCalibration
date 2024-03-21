function [FF] = FilmDrawRobot_FrameMotion(PosePList,PoseFrameList,obj)
 fig=figure
 hold on
 axis('equal')
    fig.WindowState = 'maximized';  

for i=1:obj.CablesNumber
    a{i}=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color','k');
    e{i}=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color','b');
    q{i}=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color','b');
    p{i}=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color','b');
    pulley{i,1}=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color','c');
    pulley{i,2}=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color','g');
    pulley{i,3}=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color','y');
end

line([0,.5],[0,0],[0,0], 'LineWidth', 1, 'Color', 'c');
line([0,0],[0,.5],[0,0], 'LineWidth', 1, 'Color', 'g');
line([0,0],[0,0],[0,.5], 'LineWidth', 1, 'Color', 'y');
% plot3(0, 0, 0, 'ro', 'MarkerSize', 2);

% %%%ORIENT
% A=[obj.EndEffector.Coord';obj.EndEffector.Coord';obj.EndEffector.Coord'];
% B=obj.EndEffector.RotMatrix';
% 
% line([A(1,1),A(1,1)+.3*B(1,1)],[A(1,2),A(1,2)+.3*B(1,2)],[A(1,3),A(1,3)+.3*B(1,3)], 'LineWidth', 1, 'Color', 'c');
% line([A(2,1),A(2,1)+.3*B(2,1)],[A(2,2),A(2,2)+.3*B(2,2)],[A(2,3),A(2,3)+.3*B(2,3)], 'LineWidth', 1, 'Color', 'g');
% line([A(3,1),A(3,1)+.3*B(3,1)],[A(3,2),A(3,2)+.3*B(3,2)],[A(3,3),A(3,3)+.3*B(3,3)], 'LineWidth', 1, 'Color', 'y');
grid on
% view(1.618924056272212e+02,9.469688175547727)
view(-1.567460460143623e+02,12.659333561107196)

for j=1:length(PosePList)
    obj = CorrectGeometry(obj,PoseFrameList(:,j));
    obj=SetPosePAndUpdateGeom(obj,PosePList(:,j));
    
    
    
    
    %plot pulley frames
    for i=1:obj.CablesNumber
%     for i=1:4
       vect1=[.5 0 0];
       vect2=[0 .5 0];
       vect3=[0 0 .5];

       Rotmatrix=[obj.Trasmission.Pulley{i}.PulleyFrame{1} obj.Trasmission.Pulley{i}.PulleyFrame{2} obj.Trasmission.Pulley{i}.PulleyFrame{3}].';
%        Rotmatrix=[obj.Trasmission.Pulley{i}.FixedFrame{1} obj.Trasmission.Pulley{i}.FixedFrame{2} obj.Trasmission.Pulley{i}.FixedFrame{3}].';
       vectpulley1=(vect1*Rotmatrix).'+obj.Trasmission.Pulley{i}.CableEnterPosition;
       vectpulley2=(vect2*Rotmatrix).'+obj.Trasmission.Pulley{i}.CableEnterPosition;
       vectpulley3=(vect3*Rotmatrix).'+obj.Trasmission.Pulley{i}.CableEnterPosition;
       
       
%         r=obj.Trasmission.Pulley{i}.r
%         teta=-pi:0.01:pi
%         x=r*cos(teta);
%         y=r*sin(teta)
%         plot3(x,y,zeros(1,numel(x)))
%        line([obj.Trasmission.Pulley{i}.CableEnterPosition(1),vectpulley1(1)],[obj.Trasmission.Pulley{i}.CableEnterPosition(2),vectpulley1(2)],[obj.Trasmission.Pulley{i}.CableEnterPosition(3),vectpulley1(3)], 'LineWidth', 1, 'Color', 'c');
%     line([obj.Trasmission.Pulley{i}.CableEnterPosition(1),vectpulley2(1)],[obj.Trasmission.Pulley{i}.CableEnterPosition(2),vectpulley2(2)],[obj.Trasmission.Pulley{i}.CableEnterPosition(3),vectpulley2(3)], 'LineWidth', 1, 'Color', 'g');
%     line([obj.Trasmission.Pulley{i}.CableEnterPosition(1),vectpulley3(1)],[obj.Trasmission.Pulley{i}.CableEnterPosition(2),vectpulley3(2)],[obj.Trasmission.Pulley{i}.CableEnterPosition(3),vectpulley3(3)], 'LineWidth', 1, 'Color', 'y');
       pulley{i,1}=drawline_animated(obj.Trasmission.Pulley{i}.CableEnterPosition,vectpulley1,pulley{i,1});
       pulley{i,2}=drawline_animated(obj.Trasmission.Pulley{i}.CableEnterPosition,vectpulley2,pulley{i,2});
       pulley{i,3}=drawline_animated(obj.Trasmission.Pulley{i}.CableEnterPosition,vectpulley3,pulley{i,3});
    
    end
    
    
    


    for i=1:obj.CablesNumber
%     for i=1:4
%         if j~=1
%         delete(pointA{i});
%         end
        
        WidthPoint{i}=obj.EndEffector.GlobalAttachPoints{i}-obj.EndEffector.RotMatrix*[0;0;.3];
    end
    
        for i=1:obj.CablesNumber

            if i==obj.CablesNumber

                

                e{i}=drawline_animated(obj.EndEffector.GlobalAttachPoints{i},obj.EndEffector.GlobalAttachPoints{1},e{i});

                q{i}=drawline_animated(WidthPoint{i},WidthPoint{1},q{i});

            else

                

                e{i}=drawline_animated(obj.EndEffector.GlobalAttachPoints{i},obj.EndEffector.GlobalAttachPoints{i+1},e{i});

                q{i}=drawline_animated(WidthPoint{i},WidthPoint{i+1},q{i});

            end
        p{i}=drawline_animated(obj.EndEffector.GlobalAttachPoints{i},WidthPoint{i},p{i});

        end
        
        for i=1:obj.CablesNumber
                a{i}=drawline_animated(obj.Trasmission.Pulley{i}.CableExitPosition,obj.EndEffector.GlobalAttachPoints{i},a{i});
        end
     
% FF(j)=getframe(gcf) ;
FF=1;
    xlim([-12.5 12.5])
    ylim([-12.5 12.5])
    zlim([-7 18])
            drawnow
%     if j~=1
%     delete(pointG);
%     delete(pointP);
%     end
% 
%     ptg=obj.RotMatrix_*obj.EndEffector.Platform.PtoG;
%     pointP=plot3(obj.EndEffector.Pose(1), obj.EndEffector.Pose(2), obj.EndEffector.Pose(3), 'ro', 'MarkerSize', 5,'Color','m');
%     pointG=plot3(obj.EndEffector.Pose(1)+ptg(1), obj.EndEffector.Pose(2)+ptg(2), obj.EndEffector.Pose(3)+ptg(3), 'ro', 'MarkerSize', 5,'Color','m');
% 
%     for i=1:obj.CablesNumber
%         pointA{i}=plot3(obj.EndEffector.GlobalAttachPoints{i}(1), obj.EndEffector.GlobalAttachPoints{i}(2), obj.EndEffector.GlobalAttachPoints{i}(3), 'ro', 'MarkerSize', 5,'Color','m');
%     end

end

end

