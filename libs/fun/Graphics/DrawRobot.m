function DrawRobot(obj)
% figure
clf
for i=1:obj.CablesNumber
    WidthPoint{i}=obj.EndEffector.GlobalAttachPoints{i}-obj.EndEffector.RotMatrix*[0;0;.3];
end

for i=1:obj.CablesNumber
    if i==obj.CablesNumber

        a{i}=animatedline(obj.Trasmission.Pulley{i}.CableExitPosition(1),obj.Trasmission.Pulley{i}.CableExitPosition(2),obj.Trasmission.Pulley{i}.CableExitPosition(3), 'LineWidth', 2);
        addpoints(a{i},obj.EndEffector.GlobalAttachPoints{i}(1),obj.EndEffector.GlobalAttachPoints{i}(2),obj.EndEffector.GlobalAttachPoints{i}(3));

        e{i}=animatedline(obj.EndEffector.GlobalAttachPoints{i}(1),obj.EndEffector.GlobalAttachPoints{i}(2),obj.EndEffector.GlobalAttachPoints{i}(3),'Color','b', 'LineWidth', 2);
        addpoints(e{i},obj.EndEffector.GlobalAttachPoints{1}(1),obj.EndEffector.GlobalAttachPoints{1}(2),obj.EndEffector.GlobalAttachPoints{1}(3));

        q{i}=animatedline(WidthPoint{i}(1),WidthPoint{i}(2),WidthPoint{i}(3),'Color','b', 'LineWidth', 2);
        addpoints(q{i},WidthPoint{1}(1),WidthPoint{1}(2),WidthPoint{1}(3));

    else

        a{i}=animatedline(obj.Trasmission.Pulley{i}.CableExitPosition(1),obj.Trasmission.Pulley{i}.CableExitPosition(2),obj.Trasmission.Pulley{i}.CableExitPosition(3), 'LineWidth', 2);
        addpoints(a{i},obj.EndEffector.GlobalAttachPoints{i}(1),obj.EndEffector.GlobalAttachPoints{i}(2),obj.EndEffector.GlobalAttachPoints{i}(3));

        e{i}=animatedline(obj.EndEffector.GlobalAttachPoints{i}(1),obj.EndEffector.GlobalAttachPoints{i}(2),obj.EndEffector.GlobalAttachPoints{i}(3),'Color','b', 'LineWidth', 2);
        addpoints(e{i},obj.EndEffector.GlobalAttachPoints{i+1}(1),obj.EndEffector.GlobalAttachPoints{i+1}(2),obj.EndEffector.GlobalAttachPoints{i+1}(3));

        q{i}=animatedline(WidthPoint{i}(1),WidthPoint{i}(2),WidthPoint{i}(3),'Color','b', 'LineWidth', 2);
        addpoints(q{i},WidthPoint{i+1}(1),WidthPoint{i+1}(2),WidthPoint{i+1}(3));
   
    end
    p{i}=animatedline(obj.EndEffector.GlobalAttachPoints{i}(1),obj.EndEffector.GlobalAttachPoints{i}(2),obj.EndEffector.GlobalAttachPoints{i}(3),'Color','b', 'LineWidth', 2);
    addpoints(p{i},WidthPoint{i}(1),WidthPoint{i}(2),WidthPoint{i}(3));
    
    end
drawnow
hold on
plot3(0, 0, 0, 'ro', 'MarkerSize', 2);
plot3(obj.EndEffector.Pose(1), obj.EndEffector.Pose(2), obj.EndEffector.Pose(3), 'ro', 'MarkerSize', 5,'Color','m');
GG=obj.EndEffector.Pose(1:3)+obj.EndEffector.RotMatrix*obj.EndEffector.Platform_Param.PtoG;
plot3(GG(1),GG(2),GG(3),'ro', 'MarkerSize', 5,'Color','b')
for i=1:obj.CablesNumber
plot3(obj.EndEffector.GlobalAttachPoints{i}(1), obj.EndEffector.GlobalAttachPoints{i}(2), obj.EndEffector.GlobalAttachPoints{i}(3), 'ro', 'MarkerSize', 5,'Color','m');
end
line([0,.5],[0,0],[0,0], 'LineWidth', 1, 'Color', 'c');
line([0,0],[0,.5],[0,0], 'LineWidth', 1, 'Color', 'g');
line([0,0],[0,0],[0,.5], 'LineWidth', 1, 'Color', 'y');
% %%%ORIENT
% A=[obj.EndEffector.Coord';obj.EndEffector.Coord';obj.EndEffector.Coord'];
% B=obj.EndEffector.RotMatrix';
% 
% line([A(1,1),A(1,1)+.3*B(1,1)],[A(1,2),A(1,2)+.3*B(1,2)],[A(1,3),A(1,3)+.3*B(1,3)], 'LineWidth', 1, 'Color', 'c');
% line([A(2,1),A(2,1)+.3*B(2,1)],[A(2,2),A(2,2)+.3*B(2,2)],[A(2,3),A(2,3)+.3*B(2,3)], 'LineWidth', 1, 'Color', 'g');
% line([A(3,1),A(3,1)+.3*B(3,1)],[A(3,2),A(3,2)+.3*B(3,2)],[A(3,3),A(3,3)+.3*B(3,3)], 'LineWidth', 1, 'Color', 'y');
grid on
% view(1.893000000000000e+02,25.869688385269143)
view(-90.853591497747956,20.099594980606373)
end

