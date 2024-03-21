function [FF] = FilmDrawRobot_CaricoSvinc_framemotion(PosePList,PoseFrameList,load_g_list,load_orient_list,baryc,Pos_robot,USV,obj,waterline)
 fig=figure
 hold on
 axis('equal')
    fig.WindowState = 'maximized';  

for i=1:obj.CablesNumber
    a{i}=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color','k');
    p{i}=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color',[0.6350, 0.0780, 0.1840]);

    e{i}=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color',[0.6350, 0.0780, 0.1840]);
    q{i}=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color',[0.6350, 0.0780, 0.1840]);
    f{i}=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color',[0.6350, 0.0780, 0.1840]);
    pulley{i,1}=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color','c');
    pulley{i,2}=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color','g');
    pulley{i,3}=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color','y');
    
end
load_line=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color','k');
vessel_1=animatedline('MaximumNumPoints',6,'LineWidth',2,'Color','b');
vessel_2=animatedline('MaximumNumPoints',6,'LineWidth',2,'Color','b');
vessel_3=animatedline('MaximumNumPoints',8,'LineWidth',2,'Color','b');


conj{1}=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color','b');
conj{2}=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color','b');

line([0,.5],[0,0],[0,0], 'LineWidth', 1, 'Color', 'c');
line([0,0],[0,.5],[0,0], 'LineWidth', 1, 'Color', 'g');
line([0,0],[0,0],[0,.5], 'LineWidth', 1, 'Color', 'y');
plot3(0, 0, 0, 'ro', 'MarkerSize', 2);
% %%%ORIENT
% A=[obj.EndEffector.Coord';obj.EndEffector.Coord';obj.EndEffector.Coord'];
% B=obj.EndEffector.RotMatrix';
% 
% line([A(1,1),A(1,1)+.3*B(1,1)],[A(1,2),A(1,2)+.3*B(1,2)],[A(1,3),A(1,3)+.3*B(1,3)], 'LineWidth', 1, 'Color', 'c');
% line([A(2,1),A(2,1)+.3*B(2,1)],[A(2,2),A(2,2)+.3*B(2,2)],[A(2,3),A(2,3)+.3*B(2,3)], 'LineWidth', 1, 'Color', 'g');
% line([A(3,1),A(3,1)+.3*B(3,1)],[A(3,2),A(3,2)+.3*B(3,2)],[A(3,3),A(3,3)+.3*B(3,3)], 'LineWidth', 1, 'Color', 'y');
grid on
% view(-73.681625749277671,13.632611464968154)
view(-63.607261146496832,0.786496815286624)
% view(-90,0)
% view([-90 90])
% view([0 0])
% % CameraPosition(-106.3072122505101,-25.600735606950916,35.12233977899059)
% xlim([22,32])
% % ylim([8.81052453980808,16.43467268188838])
% ylim([8,16])
% % zlim([-0.045797419648117,11.325736129339182])
% zlim([-2,10])
xlim([Pos_robot(1)-6,Pos_robot(1)+6])
% ylim([8.81052453980808,16.43467268188838])
ylim([Pos_robot(2)-6,Pos_robot(2)+6])
% zlim([-0.045797419648117,11.325736129339182])
zlim([-3,Pos_robot(3)+3])
% view(-95.838268435967223,24.734146682535165)
% view(-1.393914632547710e+02,28.142299567751614)


y=ylim;
x=xlim;


patchwater1=patch([x(2) x(1) x(1) x(2)], [y(2) y(2) y(1) y(1)], [waterline waterline waterline waterline], [ 0.5843 0.8157 0.9882])
patchwater2=patch([x(1) x(1) x(1) x(1)], [y(2) y(2) y(1) y(1)], [waterline waterline-10 waterline-10 waterline], [ 0.5843 0.8157 0.9882])
% patchwater3=patch([x(2) x(2) x(2) x(2)], [y(2) y(2) y(1) y(1)], [waterline waterline-10 waterline-10 waterline], [ 0.5843 0.8157 0.9882])
patchwater4=patch([x(1) x(2) x(2) x(1)], [y(1) y(1) y(1) y(1)], [waterline waterline waterline-10 waterline-10], [ 0.5843 0.8157 0.9882])
% patchwater5=patch([x(1) x(2) x(2) x(1)], [y(2) y(2) y(2) y(2)], [waterline waterline waterline-10 waterline-10], [ 0.5843 0.8157 0.9882])
% patchwater6=patch([x(1) x(1) x(1) x(1)], [y(2) y(2) y(1) y(1)], [waterline waterline waterline-10 waterline-10], [ 0.5843 0.8157 0.9882])
for j=1:length(PosePList)
    obj = CorrectGeometry(obj,PoseFrameList(:,j));
    obj=obj.SetPoseAndUpdate0KIN(PosePList(:,j));
    Rotmatrix_vessel= RotMatrixTaitBryan(load_orient_list(:,j));
    vessel_points(:,1)=(load_g_list(:,j)+Rotmatrix_vessel*[USV.L/2;USV.W/2;USV.Height/2]);
    vessel_points(:,2)=(load_g_list(:,j)+Rotmatrix_vessel*[-USV.L/2;USV.W/2;USV.Height/2]);
    vessel_points(:,3)=load_g_list(:,j)+Rotmatrix_vessel*([-USV.L/2;-USV.W/2;USV.Height/2]);
    vessel_points(:,4)=(load_g_list(:,j)+Rotmatrix_vessel*[USV.L/2;-USV.W/2;USV.Height/2]);
j    
    %plot pulley frames
    for i=1:obj.CablesNumber
%     for i=5
       vect1=[.5 0 0];
       vect2=[0 .5 0];
       vect3=[0 0 .5];
        
       
       Rotmatrix=[obj.Trasmission.Pulley{i}.PulleyFrame{1} obj.Trasmission.Pulley{i}.PulleyFrame{2} obj.Trasmission.Pulley{i}.PulleyFrame{3}].';
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
    
    


%     for i=1:obj.CablesNumber
    for i=1:4
%         if j~=1
%         delete(pointA{i});
%         end
%         
        WidthPoint{i}=obj.EndEffector.GlobalAttachPoints{i}-obj.EndEffector.RotMatrix*[0;0;.5+.1*(-1)^(i+1)];
    end
     conj{1}=  drawline_animated((vessel_points(:,1)+vessel_points(:,2))./2,obj.EndEffector.Pose(1:3),conj{1});
conj{2}=  drawline_animated((vessel_points(:,3)+vessel_points(:,4))./2,obj.EndEffector.Pose(1:3),conj{2});
    for i=1:obj.CablesNumber-1

        if i==obj.CablesNumber-1

            f{i}=drawline_animated(obj.EndEffector.GlobalAttachPoints{i},obj.EndEffector.GlobalAttachPoints{5},f{i});

            e{i}=drawline_animated(obj.EndEffector.GlobalAttachPoints{i},obj.EndEffector.GlobalAttachPoints{1},e{i});

            q{i}=drawline_animated(WidthPoint{i},WidthPoint{1},q{i});

        else

            f{i}=drawline_animated(obj.EndEffector.GlobalAttachPoints{i},obj.EndEffector.GlobalAttachPoints{5},f{i});

            e{i}=drawline_animated(obj.EndEffector.GlobalAttachPoints{i},obj.EndEffector.GlobalAttachPoints{i+1},e{i});

            q{i}=drawline_animated(WidthPoint{i},WidthPoint{i+1},q{i});

        end
    p{i}=drawline_animated(obj.EndEffector.GlobalAttachPoints{i},WidthPoint{i},p{i});
    load_line=drawline_animated(load_g_list(:,j),obj.EndEffector.Pose(1:3),load_line);
    end

    for i=1:obj.CablesNumber
%         for i = 5
            a{i}=drawline_animated(obj.Trasmission.Pulley{i}.CableExitPosition,obj.EndEffector.GlobalAttachPoints{i},a{i});
    end
    
     rotheight=Rotmatrix_vessel*[0;0;USV.Height];
    
    vessel_1=drawline_animated(vessel_points(:,1),vessel_points(:,2),vessel_1);
    vessel_1=drawline_animated(vessel_points(:,3),vessel_points(:,4),vessel_1);
    vessel_1=drawline_animated(vessel_points(:,4),vessel_points(:,1),vessel_1);
%     vessel_1=drawline_animated(vessel_points(:,4),vessel_points(:,1),vessel_1);
    
    vessel_2=drawline_animated(vessel_points(:,1)-Rotmatrix_vessel*[0;0;USV.Height],vessel_points(:,2)-Rotmatrix_vessel*[0;0;USV.Height],vessel_2);
    vessel_2=drawline_animated(vessel_points(:,3)-Rotmatrix_vessel*[0;0;USV.Height],vessel_points(:,4)-Rotmatrix_vessel*[0;0;USV.Height],vessel_2);
    vessel_2=drawline_animated(vessel_points(:,4)-Rotmatrix_vessel*[0;0;USV.Height],vessel_points(:,1)-Rotmatrix_vessel*[0;0;USV.Height],vessel_2);
%     vessel_2=drawline_animated(vessel_points(:,4)-Rotmatrix_vessel*[0;0;1],vessel_points(:,1)-Rotmatrix_vessel*[0;0;1],vessel_2);
    
    vessel_3=drawline_animated(vessel_points(:,1)-Rotmatrix_vessel*[0;0;USV.Height],vessel_points(:,1),vessel_3);
    vessel_3=drawline_animated(vessel_points(:,2),vessel_points(:,2)-Rotmatrix_vessel*[0;0;USV.Height],vessel_3);
    vessel_3=drawline_animated(vessel_points(:,3)-Rotmatrix_vessel*[0;0;USV.Height],vessel_points(:,3),vessel_3);
    vessel_3=drawline_animated(vessel_points(:,4),vessel_points(:,4)-Rotmatrix_vessel*[0;0;USV.Height],vessel_3);
    
     permut1=[1 0 0 0;0 1 0 0;0 0 0 1;0 0 1 0];
    patch1=patch(vessel_points(1,:).',vessel_points(2,:).',vessel_points(3,:).', [ 0.5 0.25 0]);
    patch2=patch(vessel_points(1,:).'-rotheight(1),vessel_points(2,:).'-rotheight(2),vessel_points(3,:).'-rotheight(3), [ 0.5 0.25 0]);
    patch3=patch([vessel_points(1,1:2),vessel_points(1,1:2)-rotheight(1)]*permut1,[vessel_points(2,1:2),vessel_points(2,1:2)-rotheight(2)]*permut1,[vessel_points(3,1:2),vessel_points(3,1:2)-rotheight(3)]*permut1, [ 0.5 0.25 0]);
     patch4=patch([vessel_points(1,3:4),vessel_points(1,3:4)-rotheight(1)]*permut1,[vessel_points(2,3:4),vessel_points(2,3:4)-rotheight(2)]*permut1,[vessel_points(3,3:4),vessel_points(3,3:4)-rotheight(3)]*permut1,[ 0.5 0.25 0]);
patch5=patch([vessel_points(1,1:3:4),vessel_points(1,1:3:4)-rotheight(1)]*permut1,[vessel_points(2,1:3:4),vessel_points(2,1:3:4)-rotheight(2)]*permut1,[vessel_points(3,1:3:4),vessel_points(3,1:3:4)-rotheight(3)]*permut1, [ 0.5 0.25 0]);
patch6=patch([vessel_points(1,2:3),vessel_points(1,2:3)-rotheight(1)]*permut1,[vessel_points(2,2:3),vessel_points(2,2:3)-rotheight(2)]*permut1,[vessel_points(3,2:3),vessel_points(3,2:3)-rotheight(3)]*permut1, [ 0.5 0.25 0]);

%     xlim([-3.8 3.8])
%     ylim([-2.8 2.8])
%     zlim([-7 1]) 
%       xlim([-17.5 17.5])
%     ylim([-17.5 17.5])
%     zlim([-7 15]) 
    FF(j)=getframe(gcf);
%     FF=0;

% [0.1855+2+0.08 0.1855+1+.08 0.1855-2+.08 0.1855-1+.08]
% line(xlim, [y(1) y(1)], [0.1855 0.1855],'Color', [ 0.5843 0.8157 0.9882],'LineWidth',2) 
% line( [x(1) x(1)], ylim, [0.1855 0.1855],'Color', [ 0.5843 0.8157 0.9882],'LineWidth',2) 

    drawnow
% 
%     
        delete(patch1)
        delete(patch2)
        delete(patch3)
        delete(patch4)
        delete(patch5)
        delete(patch6)
        
    if j~=1
        delete(pointB)
%     delete(pointG);
%     delete(pointP);
%     delete(pointL);
    end
%     
%     ptg=obj.RotMatrix_*obj.EndEffector.Platform.PtoG;
%     pointP=plot3(obj.EndEffector.Pose(1), obj.EndEffector.Pose(2), obj.EndEffector.Pose(3), 'ro', 'MarkerSize', 5,'Color','m');
%     pointG=plot3(obj.EndEffector.Pose(1)+ptg(1), obj.EndEffector.Pose(2)+ptg(2), obj.EndEffector.Pose(3)+ptg(3), 'ro', 'MarkerSize', 5,'Color','m');
%     pointL=plot3(load_g_list(1,j), load_g_list(2,j), load_g_list(3,j), 'ro', 'MarkerSize', 5,'Color','m');
pointB=plot3(baryc(1,j),baryc(2,j),baryc(3,j),'ro', 'MarkerSize', 5,'Color','b');
%     for i=1:obj.CablesNumber-1
%         pointA{i}=plot3(obj.EndEffector.GlobalAttachPoints{i}(1), obj.EndEffector.GlobalAttachPoints{i}(2), obj.EndEffector.GlobalAttachPoints{i}(3), 'ro', 'MarkerSize', 5,'Color','m');
%     end

end

end

